<h3>Secuencia de comandos para enlazar un repositorio local desde la consola de linux a Github</h3>
<h4>Desde cero si no se ha enlazado anteriormente</h4>
<ol>
  <li>  git init</li>
  <li>  git config --global init.defaultBranch main</li>
  <li>  git branch -m main</li>
  <li>  git branch</li>
  <li>  git add .</li>
  <li>  git remote add origin "url del repositorio https"</li>
  <li>  git commit -m "Mensaje del commit"</li>
  <li>  git config --global user.name "Nombre de usuario de github"</li>
  <li>  git config --global user.email "Correo de la cuenta de github"</li>
  <li>  git commit -m "Mensaje del commit"</li>
  <li>  git branch</li>
  <li>  ls ~/.ssh</li>
  <li>  ssh-keygen -t ed25519 -C "Correo de la cuenta de github"</li>
  <li>  cat ~/.ssh/id_ed25519.pub</li>
  <li>  ssh -T git@github.com</li>
  <p>Escribir "y" y "yes" para continuar</p>
  <li>  git remote set-url origin git@github.com:robertovazquezmaestre/Robotica_g7_25-26.git
</li>
  <p>Comprobar si la url es esa o hay que generarla con una clave ssh</p>
  <li>  git remote -v</li>
  <li>  git push -u origin "nombre del branch que quieras"</li>
</ol>
