#!/usr/bin/python3
# -*- coding: utf-8 -*-

from PySide6.QtCore import QTimer, Slot
from PySide6.QtWidgets import QApplication
from genericworker import *
import interfaces as ifaces
import numpy as np
import cv2
import torch
import os
import sys
import time

# --- IMPORTANTE: Importar el módulo generado para poder usar RoboCompMNIST.TData ---
import RoboCompMNIST

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.last_prediction = -1
        self.last_x = -1  # Nueva variable para almacenar la posición X

        if not startup_check:
            self.connect_camera()

        self.timer.timeout.connect(self.compute)
        self.timer.start(self.Period)

    def connect_camera(self):
        started = False
        while not started:
            try:
                self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                started = True
            except:
                time.sleep(1)

    @Slot()
    def compute(self):
        try:
            if not hasattr(self, 'model'):
                self.load_network()

            image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
            color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3).copy()

            # --- 1. DETECCIÓN CON FILTRO DE CENTRADO ---
            rect = self.detect_centered_frame(color)

            if rect is None:
                self.last_prediction = -1
                self.last_x = -1  # Resetear X si no hay marco
                cv2.imshow("Camera360RGB", color)
                cv2.waitKey(1)
                return

            x1, y1, x2, y2 = rect

            # --- CAPTURA DE LA POSICIÓN X ---
            # Guardamos el centro horizontal del marco detectado
            self.last_x = x1 + (x2 - x1) // 2

            roi = color[y1:y2, x1:x2]
            if roi.size == 0: return

            # --- 2. PROCESADO DEL NÚMERO ---
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 10)

            h_roi, w_roi = th.shape
            margin = int(w_roi * 0.15)
            th[0:margin, :] = 0
            th[h_roi-margin:h_roi, :] = 0
            th[:, 0:margin] = 0
            th[:, w_roi-margin:w_roi] = 0

            contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            valid_number = False
            if contours:
                cnt = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(cnt)
                x, y, w_n, h_n = cv2.boundingRect(cnt)

                if area > (w_roi * h_roi * 0.05) and h_n > (h_roi * 0.3):
                    valid_number = True

            if not valid_number:
                self.last_prediction = -1
            else:
                # --- 3. PREPARACIÓN MNIST ---
                aspect_ratio = h_n / float(w_n) if w_n > 0 else 0
                digit_img = th[y:y+h_n, x:x+w_n]
                size = max(w_n, h_n) + 10
                square = np.zeros((size, size), dtype=np.uint8)
                xo, yo = (size - w_n)//2, (size - h_n)//2
                square[yo:yo+h_n, xo:xo+w_n] = digit_img

                digit_20 = cv2.resize(square, (20, 20))
                final_input = np.zeros((28, 28), dtype=np.uint8)
                final_input[4:24, 4:24] = digit_20

                # --- 4. INFERENCIA ---
                tensor = torch.tensor(final_input, dtype=torch.float32) / 255.0
                tensor = (tensor - 0.1307) / 0.3081
                tensor = tensor.unsqueeze(0).unsqueeze(0).to(self.device)

                with torch.no_grad():
                    output = self.model(tensor)
                    prob = torch.nn.functional.softmax(output, dim=1)
                    confidence, pred = torch.max(prob, 1)

                if confidence.item() < 0.85:
                    self.last_prediction = -1
                else:
                    if pred.item() == 2 and aspect_ratio > 1.6:
                        self.last_prediction = 1
                    else:
                        self.last_prediction = pred.item()

            # --- 5. DIBUJO ---
            if self.last_prediction != -1:
                cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Dibujamos una línea vertical azul en el centro calculado para feedback visual
                cv2.line(color, (self.last_x, y1), (self.last_x, y2), (255, 0, 0), 2)
                cv2.putText(color, f"Num: {self.last_prediction} X: {self.last_x}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Camera360RGB", color)
            cv2.waitKey(1)

        except Exception as e:
            print(f"Error: {e}")

    def detect_centered_frame(self, color):
        """Busca el marco pero solo acepta los que estén cerca del centro horizontal"""
        h, w, _ = color.shape
        img_center_x = w // 2

        roi_y1, roi_y2 = int(h*0.25), int(h*0.75)
        roi_x1, roi_x2 = int(w*0.1), int(w*0.9)
        search_area = color[roi_y1:roi_y2, roi_x1:roi_x2]

        gray = cv2.cvtColor(search_area, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, bin_img = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        max_a = 0
        max_center_error = w * 0.12

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 1200 < area < 25000:
                x, y, bw, bh = cv2.boundingRect(cnt)
                obj_center_x = roi_x1 + x + (bw // 2)
                offset = abs(obj_center_x - img_center_x)

                if offset < max_center_error:
                    aspect = bw / float(bh)
                    if 0.6 < aspect < 1.6:
                        if area > max_a:
                            max_a = area
                            mx, my = int(bw*0.18), int(bh*0.18)
                            best = [roi_x1+x+mx, roi_y1+y+my, roi_x1+x+bw-mx, roi_y1+y+bh-my]
        return best

    def load_network(self):
        # ... (Mantener igual que el original) ...
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        class SimpleMNISTNet(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.conv1 = torch.nn.Conv2d(1, 16, 3, padding=1)
                self.conv2 = torch.nn.Conv2d(16, 32, 3, padding=1)
                self.pool = torch.nn.MaxPool2d(2, 2)
                self.relu = torch.nn.ReLU()
                self.fc = torch.nn.Linear(32 * 7 * 7, 10)
            def forward(self, x):
                x = self.pool(self.relu(self.conv1(x)))
                x = self.pool(self.relu(self.conv2(x)))
                x = x.view(x.size(0), -1)
                return self.fc(x)

        self.model = SimpleMNISTNet().to(self.device)
        model_path = os.path.join(os.path.dirname(__file__), "my_network.pt")
        if os.path.exists(model_path):
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

    # --- MÉTODO ACTUALIZADO PARA DEVOLVER LA ESTRUCTURA TData ---
    def MNIST_getNumber(self, c=None):
        """Implementación de la interfaz Ice que devuelve TData"""
        res = RoboCompMNIST.TData()
        res.num = int(self.last_prediction)
        res.x = int(self.last_x)
        return res