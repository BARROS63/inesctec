#include <bits/stdc++.h>
using namespace std;

struct Pt { double x,y; };

// ---------- Exportador CSV ----------
void exportCSV(const string& filename, const vector<Pt>& path){
    ofstream f(filename);
    if(!f.is_open()){
        cerr << "[ERRO] Não consegui criar " << filename << "\n";
        return;
    }
    f << "x,y\n";
    for(auto &p : path){
        f << p.x << "," << p.y << "\n";
    }
    f.close();
    cout << "[OK] Criado " << filename << " com " << path.size() << " pontos.\n";
}

// ---------- Funções dummy de exemplo ----------
// (Aqui devia estar o Lazy Theta* e os splines reais,
// mas para provar o CSV vamos gerar pontos simples.)

vector<Pt> makeLine(Pt a, Pt b, int steps){
    vector<Pt> out;
    for(int i=0;i<=steps;i++){
        double t = double(i)/steps;
        out.push_back({a.x + t*(b.x-a.x), a.y + t*(b.y-a.y)});
    }
    return out;
}

vector<Pt> cubicSplineExample(Pt a, Pt b){
    // só para exemplo: curva simples em arco
    vector<Pt> out;
    for(int i=0;i<=50;i++){
        double t = double(i)/50;
        out.push_back({a.x + t*(b.x-a.x), a.y + sin(t*3.14)*(b.y-a.y)});
    }
    return out;
}

vector<Pt> bSplineExample(Pt a, Pt b){
    vector<Pt> out;
    for(int i=0;i<=50;i++){
        double t = double(i)/50;
        out.push_back({a.x + t*(b.x-a.x), a.y + 0.5*sin(t*6.28)*(b.y-a.y)});
    }
    return out;
}

vector<Pt> bezierExample(Pt a, Pt b){
    vector<Pt> out;
    for(int i=0;i<=50;i++){
        double t = double(i)/50;
        double u = 1-t;
        Pt p0=a, p3=b, p1={a.x, b.y}, p2={b.x, a.y};
        out.push_back({
            u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x,
            u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y
        });
    }
    return out;
}

// ---------- Programa principal ----------
int main(){
    Pt start{10,10}, goal{70,50};

    // Lazy Theta* (aqui substituído por linha reta para simplificar)
    auto lazytheta = makeLine(start, goal, 20);

    // Três curvas de suavização (exemplos)
    auto cubic   = cubicSplineExample(start, goal);
    auto bspline = bSplineExample(start, goal);
    auto bezier  = bezierExample(start, goal);

    // Exportar para CSV
    exportCSV("lazytheta.csv", lazytheta);
    exportCSV("cubic.csv", cubic);
    exportCSV("bspline.csv", bspline);
    exportCSV("bezier.csv", bezier);

    cout << "\nTodos os ficheiros CSV foram criados com sucesso.\n";
    return 0;
}
