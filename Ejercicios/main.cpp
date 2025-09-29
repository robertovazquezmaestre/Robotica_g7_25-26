#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <map>
#include <thread>
#include <random>

struct Node{ int id; std::vector<int> links;} ;

void imprimir(const std::function<float ()>& func)
{
    std::cout << func() << std::endl;
}
void lambdasChronoAndTuples()
{
    //Vector de 10000000 de elementos de tuplas que se ordenan mediante sort y se usa chrono para saber
    // el tiempo que tarda en ordenarse
    /*
    std::vector<std::tuple<std::string,int>> v;
    for (int i = 0; i < 10000000; i++)
    {
        int n = rand() % 10000000;
        v.push_back(std::make_tuple(" ", n));
    }
    for (auto &it : v)
    {
        std::cout << std::get<1>(it) << std::endl;
    }
    const auto start = std::chrono::high_resolution_clock::now();
    std::ranges::sort(v,[](auto p1, auto p2){return std::get<1>(p1) < std::get<1>(p2);});
    const auto end = std::chrono::high_resolution_clock::now();
    for (auto &it : v)
    {
        std::cout << std::get<1>(it) << std::endl;
    }
    std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << std::endl;
    */
    //Calcular gradientes
    auto g=180;
    imprimir([g](){return g*(3.14/180.0);});

}

void ejercicio_threads()
{
    // Create and start a thread
    std::thread t1([](int g) {std::cout << g << std::endl;},9);
    // Create and start a lambda thread with parameter n
    std::thread t3([](int n) {std::cout << n << std::endl;},3);

    t1.join();
    t3.join();

}

void aleatorio()
{
    std::random_device rd;
     std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dice(1, 6);
    std::uniform_real_distribution<double> percentage(0.0, 100.0);
    std::normal_distribution<double> height(170.0, 10.0);
    // Height: mean=170cm, std=10cm
    std::cout << "=== Random Examples ===\n";
    // Generate some dice rolls
    std::cout << "Dice rolls: ";
    for (int i = 0; i < 5; ++i)
    { std::cout <<  dice(gen) << " ";
        std::cout << percentage(gen);
        std::cout << height(gen);
    }
    std::cout << "\n";
}

void large_vector()
{
    int elementos=(1024*1024*1024)/4;
    std::vector<int> v(elementos);
    auto porcopia=[](std::vector<int> v){};
    auto start = std::chrono::high_resolution_clock::now();
    porcopia(v);
    auto end = std::chrono::high_resolution_clock::now();
    auto total=std::chrono::duration_cast<std::chrono::seconds>(end-start);
    std::cout << total.count() << " seconds\n";
    auto porreferencia=[](std::vector<int>& v){};
    auto start2 = std::chrono::high_resolution_clock::now();
    porreferencia(v);
    auto end2 = std::chrono::high_resolution_clock::now();
    auto total2=std::chrono::duration_cast<std::chrono::seconds>(end-start);
    std::cout << total2.count() << " seconds\n";


}

void grafo()
{
    std::map<int, Node> graph;
    for(int i=0; i<100; i++)
        graph.emplace(i, Node{i, std::vector<int>()});
    for(auto &[key, value] : graph)
    {
        auto vecinos = rand()%5;
        for(int j=0; j<vecinos; j++)
            value.links.emplace_back( rand()%99);
    }

}

int main()
{

   //lambdasChronoAndTuples();
   //ejercicio_threads();
    //large_vector();
    aleatorio();
    return 0;
}