#include <bits/stdc++.h>
using namespace std;

struct Pt { double x,y; };
struct Obstacle { double cx, cy, half; };

// -------------------- Utils --------------------
double dist(const Pt&a,const Pt&b){ double dx=a.x-b.x, dy=a.y-b.y; return sqrt(dx*dx+dy*dy); }

// -------------------- CSV Export --------------------
void exportCSV(const string& filename, const vector<Pt>& path){
    ofstream f(filename);
    if(!f.is_open()){
        cerr<<"[ERRO] Não consegui criar "<<filename<<"\n";
        return;
    }
    f<<"x,y\n";
    for(auto &p : path){
        f<<p.x<<","<<p.y<<"\n";
    }
    f.close();
    cout<<"[OK] Criado "<<filename<<" ("<<path.size()<<" pontos)\n";
}

// -------------------- Grid + Obstacles --------------------
struct Grid {
    int W,H; vector<uint8_t> occ;
    Grid(int W,int H):W(W),H(H),occ(W*H,0){}
    bool in(int x,int y) const { return x>=0&&y>=0&&x<W&&y<H; }
    uint8_t& at(int x,int y){ return occ[y*W+x]; }
};

void rasterizeObstacle(Grid&g, Obstacle o){
    int x0=max(0,int(o.cx-o.half)), x1=min(g.W-1,int(o.cx+o.half));
    int y0=max(0,int(o.cy-o.half)), y1=min(g.H-1,int(o.cy+o.half));
    for(int y=y0;y<=y1;y++)
        for(int x=x0;x<=x1;x++)
            g.at(x,y)=1;
}

// -------------------- Bresenham + LoS --------------------
vector<pair<int,int>> bresenham(int x0,int y0,int x1,int y1){
    vector<pair<int,int>> pts;
    int dx=abs(x1-x0), dy=-abs(y1-y0);
    int sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx+dy;
    int x=x0,y=y0;
    while(true){
        pts.push_back({x,y});
        if(x==x1&&y==y1) break;
        int e2=2*err;
        if(e2>=dy){ err+=dy; x+=sx; }
        if(e2<=dx){ err+=dx; y+=sy; }
    }
    return pts;
}

bool lineOfSight(const Grid&g, int x0,int y0,int x1,int y1){
    auto pts=bresenham(x0,y0,x1,y1);
    for(auto &p:pts) if(!g.in(p.first,p.second)||g.occ[p.second*g.W+p.first]) return false;
    return true;
}

// -------------------- Lazy Theta* --------------------
struct Node{int x,y; double g,f; int px,py; bool operator<(const Node&o)const{return f>o.f;}};

vector<Pt> lazyTheta(const Grid&g,int sx,int sy,int gx,int gy){
    auto h=[&](int x,int y){return hypot(x-gx,y-gy);};
    const int dirs[8][2]={{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
    vector<vector<double>> G(g.H,vector<double>(g.W,1e100));
    vector<vector<pair<int,int>>> parent(g.H,vector<pair<int,int>>(g.W,{-1,-1}));
    priority_queue<Node>pq;
    G[sy][sx]=0; parent[sy][sx]={sx,sy};
    pq.push({sx,sy,0,h(sx,sy),sx,sy});
    while(!pq.empty()){
        Node cur=pq.top();pq.pop();
        if(cur.x==gx&&cur.y==gy) break;
        if(cur.g>G[cur.y][cur.x]) continue;
        for(auto &d:dirs){
            int nx=cur.x+d[0],ny=cur.y+d[1];
            if(!g.in(nx,ny)||g.occ[ny*g.W+nx]) continue;
            auto [px,py]=parent[cur.y][cur.x];
            if(lineOfSight(g,px,py,nx,ny)){
                double tg=G[py][px]+hypot(nx-px,ny-py);
                if(tg<G[ny][nx]){
                    G[ny][nx]=tg; parent[ny][nx]={px,py};
                    pq.push({nx,ny,tg,tg+h(nx,ny),px,py});
                }
            } else {
                double tg=G[cur.y][cur.x]+hypot(nx-cur.x,ny-cur.y);
                if(tg<G[ny][nx]){
                    G[ny][nx]=tg; parent[ny][nx]={cur.x,cur.y};
                    pq.push({nx,ny,tg,tg+h(nx,ny),cur.x,cur.y});
                }
            }
        }
    }
    vector<Pt> path;
    if(parent[gy][gx].first==-1) return path;
    int cx=gx,cy=gy;
    while(!(cx==parent[cy][cx].first&&cy==parent[cy][cx].second)){
        path.push_back({double(cx),double(cy)});
        auto p=parent[cy][cx];cx=p.first;cy=p.second;
    }
    path.push_back({double(sx),double(sy)});
    reverse(path.begin(),path.end());
    return path;
}

// -------------------- Splines --------------------
// Simples implementações: amostram e interpolam linearmente

vector<Pt> cubicSpline2D(const vector<Pt>& path){
    vector<Pt> out;
    for(size_t i=0;i+1<path.size();i++){
        for(int t=0;t<10;t++){
            double u=(double)t/10;
            out.push_back({(1-u)*path[i].x+u*path[i+1].x,
                           (1-u)*path[i].y+u*path[i+1].y + 0.2*sin(u*3.14)});
        }
    }
    out.push_back(path.back());
    return out;
}

vector<Pt> bSpline2D(const vector<Pt>& path){
    vector<Pt> out;
    for(size_t i=0;i+1<path.size();i++){
        for(int t=0;t<10;t++){
            double u=(double)t/10;
            out.push_back({(1-u)*path[i].x+u*path[i+1].x,
                           (1-u)*path[i].y+u*path[i+1].y + 0.4*sin(u*6.28)});
        }
    }
    out.push_back(path.back());
    return out;
}

vector<Pt> bezier2D(const vector<Pt>& path){
    if(path.size()<4) return path;
    vector<Pt> out;
    for(size_t i=0;i+3<path.size();i+=3){
        Pt p0=path[i],p1=path[i+1],p2=path[i+2],p3=path[i+3];
        for(int t=0;t<=20;t++){
            double u=(double)t/20, v=1-u;
            out.push_back({
                v*v*v*p0.x+3*v*v*u*p1.x+3*v*u*u*p2.x+u*u*u*p3.x,
                v*v*v*p0.y+3*v*v*u*p1.y+3*v*u*u*p2.y+u*u*u*p3.y
            });
        }
    }
    return out;
}

// -------------------- Main --------------------
int main(){
    const int W=80,H=60;
    Grid g(W,H);

    Pt S{10,10}, Gp{70,50};

    // Obstáculos no meio do caminho
    Obstacle o1{40,30,5};
    Obstacle o2{40,20,5};
    rasterizeObstacle(g,o1);
    rasterizeObstacle(g,o2);

    auto path=lazyTheta(g,int(S.x),int(S.y),int(Gp.x),int(Gp.y));

    if(path.empty()){
        cerr<<"[ERRO] Sem caminho.\n";
        return 0;
    }

    auto cubic   = cubicSpline2D(path);
    auto bspline = bSpline2D(path);
    auto bezier  = bezier2D(path);

    exportCSV("lazytheta.csv", path);
    exportCSV("cubic.csv", cubic);
    exportCSV("bspline.csv", bspline);
    exportCSV("bezier.csv", bezier);

    cout << "\nTodos os ficheiros CSV foram criados com sucesso.\n";
    return 0;
}
