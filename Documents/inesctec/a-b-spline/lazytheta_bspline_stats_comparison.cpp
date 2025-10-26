#include <bits/stdc++.h>
using namespace std;

struct Pt { double x,y; };
struct Obstacle { double cx, cy, half; };

double dist(const Pt&a,const Pt&b){ double dx=a.x-b.x, dy=a.y-b.y; return sqrt(dx*dx+dy*dy); }

void exportCSV(const string& filename, const vector<Pt>& path){
    ofstream f(filename);
    f<<"x,y\n";
    for(auto&p:path) f<<p.x<<","<<p.y<<"\n";
    cout<<"[OK] "<<filename<<" ("<<path.size()<<" pts)\n";
}

struct Grid {
    int W,H; vector<uint8_t> occ;
    Grid(int W,int H):W(W),H(H),occ(W*H,0){}
    bool in(int x,int y) const {return x>=0&&y>=0&&x<W&&y<H;}
    uint8_t& at(int x,int y){return occ[y*W+x];}
};

void rasterizeObstacle(Grid&g, const Obstacle&o){
    int x0=max(0,int(o.cx-o.half)), x1=min(g.W-1,int(o.cx+o.half));
    int y0=max(0,int(o.cy-o.half)), y1=min(g.H-1,int(o.cy+o.half));
    for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) g.at(x,y)=1;
}

vector<pair<int,int>> bresenham(int x0,int y0,int x1,int y1){
    vector<pair<int,int>> pts;
    int dx=abs(x1-x0), dy=-abs(y1-y0), sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx+dy;
    int x=x0,y=y0;
    while(true){
        pts.push_back({x,y});
        if(x==x1 && y==y1) break;
        int e2=2*err;
        if(e2>=dy){ err+=dy; x+=sx; }
        if(e2<=dx){ err+=dx; y+=sy; }
    }
    return pts;
}

bool lineOfSight(const Grid&g, int x0,int y0,int x1,int y1){
    for(auto&p:bresenham(x0,y0,x1,y1)){
        if(!g.in(p.first,p.second)||g.occ[p.second*g.W+p.first]) return false;
    }
    return true;
}

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
                if(tg<G[ny][nx]){G[ny][nx]=tg; parent[ny][nx]={px,py}; pq.push({nx,ny,tg,tg+h(nx,ny),px,py});}
            } else {
                double tg=G[cur.y][cur.x]+hypot(nx-cur.x,ny-cur.y);
                if(tg<G[ny][nx]){G[ny][nx]=tg; parent[ny][nx]={cur.x,cur.y}; pq.push({nx,ny,tg,tg+h(nx,ny),cur.x,cur.y});}
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

//=========== Natural cubic spline 1D ===========
struct Cubic1D{
    vector<double> t,a,b,c,d;
    void build(const vector<double>&T,const vector<double>&V){
        int n=T.size(); t=T; a=V; b.assign(n,0); c.assign(n,0); d.assign(n,0);
        if(n<2) return;
        vector<double> h(n-1),alpha(n-1);
        for(int i=0;i<n-1;i++) h[i]=T[i+1]-T[i];
        vector<double> l(n),mu(n),z(n);
        l[0]=1; mu[0]=z[0]=0;
        for(int i=1;i<n-1;i++){
            alpha[i]=(3.0/h[i])*(a[i+1]-a[i])-(3.0/h[i-1])*(a[i]-a[i-1]);
            l[i]=2*(T[i+1]-T[i-1])-h[i-1]*mu[i-1];
            mu[i]=h[i]/l[i]; z[i]=(alpha[i]-h[i-1]*z[i-1])/l[i];
        }
        l[n-1]=1; z[n-1]=0; c[n-1]=0;
        for(int j=n-2;j>=0;j--){c[j]=z[j]-mu[j]*c[j+1]; b[j]=(a[j+1]-a[j])/h[j]-h[j]*(c[j+1]+2*c[j])/3; d[j]=(c[j+1]-c[j])/(3*h[j]);}
    }
    double eval(double X)const{
        int n=t.size(); if(n==0) return 0; if(n==1) return a[0];
        int i=int(upper_bound(t.begin(),t.end(),X)-t.begin())-1;
        i=max(0,min(i,n-2)); double dt=X-t[i];
        return a[i]+b[i]*dt+c[i]*dt*dt+d[i]*dt*dt*dt;
    }
};

vector<Pt> splineCubic2D(const vector<Pt>& P,int samplesPerSeg=30){
    int n=P.size(); if(n<2) return P;
    vector<double> T(n,0.0);
    for(int i=1;i<n;i++) T[i]=T[i-1]+dist(P[i],P[i-1]);
    vector<double>X(n),Y(n); for(int i=0;i<n;i++){X[i]=P[i].x;Y[i]=P[i].y;}
    Cubic1D Sx,Sy; Sx.build(T,X); Sy.build(T,Y);
    vector<Pt> out; int total=(n-1)*samplesPerSeg;
    for(int k=0;k<=total;k++){double u=T.front()+(T.back()-T.front())*k/double(total);
        out.push_back({Sx.eval(u),Sy.eval(u)});}
    return out;
}

//=========== B-spline (de Boor) ===========
vector<double> uniformClampedKnot(int m,int p){
    int K=m+p+1; vector<double>U(K);
    for(int i=0;i<=p;i++) U[i]=0.0;
    for(int i=p+1;i<K-p-1;i++) U[i]=double(i-p)/double(K-2*p-1);
    for(int i=K-p-1;i<K;i++) U[i]=1.0; return U;
}
Pt deBoorCubic(const vector<Pt>&C,const vector<double>&U,double u){
    const int p=3; int m=U.size()-1,n=C.size()-1;
    if(u>=U[m-p-1]) return C.back(); int i=p; while(i<m-p-1 && !(u>=U[i]&&u<U[i+1])) ++i;
    vector<Pt>d(p+1); for(int r=0;r<=p;r++) d[r]=C[i-p+r];
    for(int r=1;r<=p;r++) for(int j=p;j>=r;--j){double denom=U[i+j-r+1]-U[i+j-p];
        double alpha=denom>1e-12?(u-U[i+j-p])/denom:0;
        d[j].x=(1-alpha)*d[j-1].x+alpha*d[j].x; d[j].y=(1-alpha)*d[j-1].y+alpha*d[j].y;}
    return d[p];
}
vector<Pt> bSplineCubic(const vector<Pt>&ctrl,int samples=400){
    if(ctrl.size()<4) return ctrl; const int p=3;
    vector<double>U=uniformClampedKnot(ctrl.size(),p); vector<Pt>out;
    for(int i=0;i<=samples;i++){double u=double(i)/samples; out.push_back(deBoorCubic(ctrl,U,u));}
    return out;
}

//=========== Piecewise Bézier ===========
vector<Pt> bezierPiecewise(const vector<Pt>&P,int samples=40){
    int n=P.size(); if(n<2) return P;
    auto add=[&](Pt a,Pt b){return Pt{a.x+b.x,a.y+b.y};};
    auto sub=[&](Pt a,Pt b){return Pt{a.x-b.x,a.y-b.y};};
    auto mul=[&](Pt a,double s){return Pt{a.x*s,a.y*s};};
    vector<Pt>T(n); for(int i=0;i<n;i++){Pt pPrev=P[max(0,i-1)],pNext=P[min(n-1,i+1)];T[i]=mul(sub(pNext,pPrev),0.5);}
    vector<Pt>out; out.push_back(P.front());
    for(int i=0;i<n-1;i++){Pt P0=P[i],P3=P[i+1];Pt P1=add(P0,mul(T[i],1.0/3)),P2=sub(P3,mul(T[i+1],1.0/3));
        for(int s=1;s<=samples;s++){double t=double(s)/samples,u=1.0-t;
            out.push_back({u*u*u*P0.x+3*u*u*t*P1.x+3*u*t*t*P2.x+t*t*t*P3.x,
                           u*u*u*P0.y+3*u*u*t*P1.y+3*u*t*t*P2.y+t*t*t*P3.y});}}
    return out;
}

//=========== Metrics ===========
struct Metrics{double length,smooth;int collisions;};
Metrics evaluate(const vector<Pt>&path,const vector<Obstacle>&obs){
    Metrics M{0,0,0}; if(path.size()<2) return M;
    for(size_t i=1;i<path.size();i++) M.length+=dist(path[i-1],path[i]);
    for(size_t i=1;i+1<path.size();i++){
        Pt a={path[i].x-path[i-1].x,path[i].y-path[i-1].y};
        Pt b={path[i+1].x-path[i].x,path[i+1].y-path[i].y};
        double dot=a.x*b.x+a.y*b.y, na=hypot(a.x,a.y), nb=hypot(b.x,b.y);
        if(na>1e-9&&nb>1e-9){double c=dot/(na*nb); c=max(-1.0,min(1.0,c)); M.smooth+=acos(c);}
    }
    for(auto&p:path) for(auto&o:obs)
        if(fabs(p.x-o.cx)<=o.half&&fabs(p.y-o.cy)<=o.half) M.collisions++;
    return M;
}

//=========== Main ===========
int main(){
    Grid g(60,60);
    Pt S{10,10},G{50,50};
    Obstacle o1{20,20,5},o2{40,40,5}; rasterizeObstacle(g,o1); rasterizeObstacle(g,o2);
    vector<Obstacle> obs={o1,o2};

    auto path=lazyTheta(g,int(S.x),int(S.y),int(G.x),int(G.y));
    auto cubic=splineCubic2D(path,30);
    auto bspline=bSplineCubic(path,600);
    auto bezier=bezierPiecewise(path,40);

    exportCSV("lazytheta.csv",path);
    exportCSV("cubic.csv",cubic);
    exportCSV("bspline.csv",bspline);
    exportCSV("bezier.csv",bezier);

    auto Mc=evaluate(cubic,obs), Mb=evaluate(bspline,obs), Mz=evaluate(bezier,obs);

    cout<<"\n=== Metrics (lower is better) ===\n";
    cout<<"Cubic:   collisions="<<Mc.collisions<<" length="<<Mc.length<<" smooth="<<Mc.smooth<<"\n";
    cout<<"B-Spline:collisions="<<Mb.collisions<<" length="<<Mb.length<<" smooth="<<Mb.smooth<<"\n";
    cout<<"Bezier:  collisions="<<Mz.collisions<<" length="<<Mz.length<<" smooth="<<Mz.smooth<<"\n";

    // winner logic
    auto best=[&](double a,double b){return (a<b)?100.0*(1-b/a):100.0*(1-a/b);};
    cout<<"\n=== Comparative gains ===\n";
    cout<<"B-Spline vs Cubic length gain: "<<100.0*(Mc.length-Mb.length)/Mc.length<<"%\n";
    cout<<"B-Spline vs Bezier length gain: "<<100.0*(Mz.length-Mb.length)/Mz.length<<"%\n";
    cout<<"B-Spline vs Cubic smoothness gain: "<<100.0*(Mc.smooth-Mb.smooth)/Mc.smooth<<"%\n";
    cout<<"B-Spline vs Bezier smoothness gain: "<<100.0*(Mz.smooth-Mb.smooth)/Mz.smooth<<"%\n";

    return 0;
}
