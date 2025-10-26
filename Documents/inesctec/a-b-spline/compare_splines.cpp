#include <bits/stdc++.h>
using namespace std;

struct Pt { double x,y; };
struct Obstacle { double cx, cy, half; }; // axis-aligned square

// -------------------- Utility --------------------
static inline double dist(const Pt&a, const Pt&b){ double dx=a.x-b.x, dy=a.y-b.y; return sqrt(dx*dx+dy*dy); }
static inline double dot2(double ax,double ay,double bx,double by){ return ax*bx+ay*by; }
static inline double norm(double x,double y){ return sqrt(x*x+y*y); }
static inline double clampd(double v,double a,double b){ return max(a,min(b,v)); }

// -------------------- Grid + Bresenham + LoS --------------------
struct Grid {
    int W,H; vector<uint8_t> occ; // 0 free, 1 occupied
    Grid(int W,int H):W(W),H(H),occ(W*H,0) {}
    bool in(int x,int y) const { return x>=0&&y>=0&&x<W&&y<H; }
    uint8_t& at(int x,int y){ return occ[y*W+x]; }
    uint8_t  at(int x,int y) const { return occ[y*W+x]; }
};

// Bresenham integer line
static vector<pair<int,int>> bresenham(int x0,int y0,int x1,int y1){
    vector<pair<int,int>> pts;
    int dx = abs(x1-x0), dy = -abs(y1-y0);
    int sx = x0<x1?1:-1; int sy = y0<y1?1:-1; int err = dx+dy;
    int x=x0,y=y0;
    for(;;){
        pts.emplace_back(x,y);
        if(x==x1 && y==y1) break;
        int e2 = 2*err;
        if(e2>=dy){ err+=dy; x+=sx; }
        if(e2<=dx){ err+=dx; y+=sy; }
    }
    return pts;
}

static bool lineOfSight(const Grid&g, int x0,int y0,int x1,int y1){
    auto pts = bresenham(x0,y0,x1,y1);
    for (auto &p: pts){
        if(!g.in(p.first,p.second) || g.at(p.first,p.second)) return false;
    }
    return true;
}

// -------------------- Lazy Theta* --------------------
struct Node {
    int x,y; double g,f; int px,py;
    bool operator<(const Node& o) const { return f>o.f; } // min-heap
};

static vector<Pt> lazyTheta(const Grid&g, int sx,int sy,int gx,int gy){
    auto h = [&](int x,int y){ return hypot(double(x-gx),double(y-gy)); };
    const int dirs[8][2] = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};

    vector<vector<double>> G(g.H, vector<double>(g.W, 1e100));
    vector<vector<pair<int,int>>> parent(g.H, vector<pair<int,int>>(g.W, {-1,-1}));
    priority_queue<Node> pq;

    G[sy][sx]=0; parent[sy][sx]={sx,sy};
    pq.push({sx,sy,0,h(sx,sy),sx,sy});

    while(!pq.empty()){
        Node cur=pq.top(); pq.pop();
        if(cur.x==gx && cur.y==gy) break;

        if(cur.g>G[cur.y][cur.x]+1e-12) continue;

        for (auto &d:dirs){
            int nx=cur.x+d[0], ny=cur.y+d[1];
            if(!g.in(nx,ny) || g.at(nx,ny)) continue;

            auto [px,py] = parent[cur.y][cur.x];

            // Try connect parent->neighbor if LoS
            if(lineOfSight(g, px,py, nx,ny)){
                double tg = G[py][px] + hypot(double(nx-px), double(ny-py));
                if(tg < G[ny][nx]){
                    G[ny][nx]=tg;
                    parent[ny][nx]={px,py};
                    double f = tg + h(nx,ny);
                    pq.push({nx,ny,tg,f,px,py});
                }
            } else { // fallback current->neighbor
                double tg = G[cur.y][cur.x] + hypot(double(nx-cur.x), double(ny-cur.y));
                if(tg < G[ny][nx]){
                    G[ny][nx]=tg;
                    parent[ny][nx]={cur.x,cur.y};
                    double f = tg + h(nx,ny);
                    pq.push({nx,ny,tg,f,cur.x,cur.y});
                }
            }
        }
    }

    // Reconstruct
    vector<Pt> path;
    if(parent[gy][gx].first==-1) return path;
    int cx=gx, cy=gy;
    while(!(cx==parent[cy][cx].first && cy==parent[cy][cx].second)){
        path.push_back(Pt{double(cx),double(cy)});
        auto p = parent[cy][cx];
        cx=p.first; cy=p.second;
    }
    path.push_back(Pt{double(sx),double(sy)});
    reverse(path.begin(), path.end());
    return path;
}

// -------------------- Obstacles construction --------------------
// Place two squares around the midpoint, offset perpendicular to Start->Goal
static vector<Obstacle> makeMidlineObstacles(Pt S, Pt G, double half=1.5, double offset=3.0){
    Pt M{ (S.x+G.x)/2.0, (S.y+G.y)/2.0 };
    double vx=G.x-S.x, vy=G.y-S.y;
    double L = norm(vx,vy); if(L<1e-9) L=1;
    // Perpendicular unit
    double nx = -vy/L, ny =  vx/L;
    Pt A{ M.x + offset*nx, M.y + offset*ny };
    Pt B{ M.x - offset*nx, M.y - offset*ny };
    return { {A.x,A.y,half}, {B.x,B.y,half} };
}

static void rasterizeObstacles(Grid&g, const vector<Obstacle>&obs){
    for (auto &o: obs){
        int x0 = max(0, int(floor(o.cx - o.half)));
        int x1 = min(g.W-1, int( ceil(o.cx + o.half)));
        int y0 = max(0, int(floor(o.cy - o.half)));
        int y1 = min(g.H-1, int( ceil(o.cy + o.half)));
        for(int y=y0;y<=y1;y++)
            for(int x=x0;x<=x1;x++)
                g.at(x,y)=1;
    }
}

static bool pointInObstacle(const vector<Obstacle>&obs, double x, double y){
    for (auto &o: obs){
        if (x>=o.cx-o.half && x<=o.cx+o.half && y>=o.cy-o.half && y<=o.cy+o.half)
            return true;
    }
    return false;
}

// -------------------- Spline: Natural Cubic (interpolating) --------------------
struct Cubic1D {
    // Natural cubic spline on (t[i], v[i]) sorted by t
    vector<double> t, a,b,c,d; // piecewise coefficients: v = a + b*dt + c*dt^2 + d*dt^3
    void build(const vector<double>&T, const vector<double>&V){
        int n = (int)T.size();
        t=T; a=V; b.assign(n,0); c.assign(n,0); d.assign(n,0);
        if(n<2) return;

        vector<double> h(n-1), alpha(n-1);
        for(int i=0;i<n-1;i++) h[i]=T[i+1]-T[i];
        vector<double> l(n), mu(n), z(n);
        l[0]=1; mu[0]=z[0]=0;
        for(int i=1;i<n-1;i++){
            alpha[i] = (3.0/h[i])*(a[i+1]-a[i]) - (3.0/h[i-1])*(a[i]-a[i-1]);
            l[i] = 2*(T[i+1]-T[i-1]) - h[i-1]*mu[i-1];
            mu[i] = h[i]/l[i];
            z[i] = (alpha[i]-h[i-1]*z[i-1]) / l[i];
        }
        l[n-1]=1; z[n-1]=0; c[n-1]=0;
        for(int j=n-2;j>=0;j--){
            c[j] = z[j] - mu[j]*c[j+1];
            b[j] = (a[j+1]-a[j])/h[j] - h[j]*(c[j+1]+2*c[j])/3.0;
            d[j] = (c[j+1]-c[j])/(3.0*h[j]);
        }
    }
    double eval(double X) const{
        int n=t.size();
        if(n==0) return 0;
        if(n==1) return a[0];
        int i = int(upper_bound(t.begin(), t.end(), X) - t.begin()) - 1;
        i = clamp(i,0,n-2);
        double dt = X - t[i];
        return a[i] + b[i]*dt + c[i]*dt*dt + d[i]*dt*dt*dt;
    }
};

static vector<Pt> cubicSpline2D(const vector<Pt>& P, int samplesPerSeg=20){
    int n=P.size(); if(n<2) return P;
    vector<double> T(n,0.0);
    for(int i=1;i<n;i++) T[i]=T[i-1]+dist(P[i],P[i-1]); // chord length parametrization
    // Avoid degenerate
    for(int i=1;i<n;i++) if (T[i]==T[i-1]) T[i]+=1e-6;

    vector<double> X(n), Y(n);
    for(int i=0;i<n;i++){ X[i]=P[i].x; Y[i]=P[i].y; }
    Cubic1D Sx, Sy; Sx.build(T,X); Sy.build(T,Y);

    vector<Pt> out;
    int totalSamples = max(2, (n-1)*samplesPerSeg);
    for(int k=0;k<=totalSamples;k++){
        double u = T.front() + (T.back()-T.front())* (double(k)/double(totalSamples));
        out.push_back( Pt{ Sx.eval(u), Sy.eval(u) } );
    }
    return out;
}

// -------------------- Uniform cubic B-spline (clamped) via De Boor --------------------
static vector<double> uniformKnot(int m, int p){
    // m = #control points, degree p=3; clamped uniform knot vector length m+p+1
    int k = m+p+1;
    vector<double> U(k);
    for(int i=0;i<=p;i++) U[i]=0.0;
    for(int i=p+1;i<k-p-1;i++) U[i] = double(i-p)/double(k-2*p-1);
    for(int i=k-p-1;i<k;i++) U[i]=1.0;
    return U;
}

static Pt deBoorCubic(const vector<Pt>&C, const vector<double>&U, double u){
    int p=3;
    int n=C.size()-1;
    // find span
    int m = U.size()-1;
    if(u>=U[m-p-1]) { // clamp right
        return C.back();
    }
    int i = p;
    while(i<m-p-1 && !(u>=U[i] && u<U[i+1])) ++i;

    // copy control points
    vector<Pt> d(p+1);
    for(int r=0;r<=p;r++) d[r]=C[i-p+r];

    for(int r=1;r<=p;r++){
        for(int j=p; j>=r; --j){
            double denom = U[i+j-r+1]-U[i+j-p];
            double alpha = (denom>1e-12)? (u - U[i+j-p])/denom : 0.0;
            d[j].x = (1-alpha)*d[j-1].x + alpha*d[j].x;
            d[j].y = (1-alpha)*d[j-1].y + alpha*d[j].y;
        }
    }
    return d[p];
}

static vector<Pt> bSplineUniformCubic(const vector<Pt>& ctrl, int samples=200){
    if(ctrl.size()<4) return ctrl;
    int p=3;
    vector<double> U = uniformKnot((int)ctrl.size(), p);
    vector<Pt> out;
    for(int i=0;i<=samples;i++){
        double u = double(i)/double(samples);
        out.push_back( deBoorCubic(ctrl, U, u) );
    }
    return out;
}

// -------------------- Piecewise cubic Bézier via Catmull–Rom-like tangents --------------------
static vector<Pt> bezierPiecewise(const vector<Pt>& P, int samplesPerSeg=30){
    int n=P.size(); if(n<2) return P;
    auto add = [&](const Pt&a,const Pt&b){ return Pt{a.x+b.x, a.y+b.y}; };
    auto sub = [&](const Pt&a,const Pt&b){ return Pt{a.x-b.x, a.y-b.y}; };
    auto mul = [&](const Pt&a,double s){ return Pt{a.x*s, a.y*s}; };

    // tangents (Catmull-Rom style)
    vector<Pt> T(n);
    for(int i=0;i<n;i++){
        Pt pPrev = P[max(0,i-1)];
        Pt pNext = P[min(n-1,i+1)];
        T[i] = mul( sub(pNext,pPrev), 0.5 );
    }

    vector<Pt> out; out.push_back(P.front());
    for(int i=0;i<n-1;i++){
        Pt P0=P[i], P3=P[i+1];
        Pt P1 = add(P0, mul(T[i], 1.0/3.0));
        Pt P2 = sub(P3, mul(T[i+1], 1.0/3.0));
        for(int s=1;s<=samplesPerSeg;s++){
            double t = double(s)/double(samplesPerSeg);
            double u = 1.0 - t;
            // cubic Bézier blending
            Pt B{
                u*u*u*P0.x + 3*u*u*t*P1.x + 3*u*t*t*P2.x + t*t*t*P3.x,
                u*u*u*P0.y + 3*u*u*t*P1.y + 3*u*t*t*P2.y + t*t*t*P3.y
            };
            out.push_back(B);
        }
    }
    return out;
}

// -------------------- Metrics --------------------
struct Metrics {
    int collisions;           // # of samples inside obstacles
    double length;            // arc length
    double smooth;            // sum of absolute heading changes (radians)
};

static Metrics measure(const vector<Pt>&C, const vector<Obstacle>&obs){
    Metrics M{0,0.0,0.0};
    if(C.size()<2) return M;
    // collisions
    for(auto &p: C) if(pointInObstacle(obs,p.x,p.y)) M.collisions++;

    // length + smoothness
    double prevHeading = NAN;
    for(size_t i=1;i<C.size();++i){
        M.length += dist(C[i-1], C[i]);
        double vx=C[i].x-C[i-1].x, vy=C[i].y-C[i-1].y;
        double h = atan2(vy,vx);
        if(!std::isnan(prevHeading)){
            double dh = h - prevHeading;
            // wrap to [-pi,pi]
            while(dh>M_PI) dh-=2*M_PI;
            while(dh<-M_PI) dh+=2*M_PI;
            M.smooth += fabs(dh);
        }
        prevHeading = h;
    }
    return M;
}

static void printMetrics(const string& name, const Metrics&M){
    cout<<fixed<<setprecision(3);
    cout<<left<<setw(18)<<name
        <<" | collisions="<<setw(4)<<M.collisions
        <<" length="<<setw(8)<<M.length
        <<" smooth="<<setw(8)<<M.smooth<<"\n";
}

static double pctBetter(double best, double other){ // lower is better
    if(other<=1e-12) return 0.0;
    return (other - best)/other * 100.0;
}

// -------------------- Main experiment --------------------
int main(){
    ios::sync_with_stdio(false);
    // --- grid and start/goal ---
    const int W=80, H=60;
    Grid g(W,H);

    // You can change these if you wish:
    Pt S{10,10};
    Pt G{70,50};

    // Build obstacles around midline (two squares)
    auto obstacles = makeMidlineObstacles(S,G, /*half*/ 3.0, /*offset*/ 4.0);
    rasterizeObstacles(g, obstacles);

    // Plan with Lazy Theta*
    auto path = lazyTheta(g, int(S.x), int(S.y), int(G.x), int(G.y));
    if(path.size()<2){
        cerr<<"No path found.\n";
        return 0;
    }

    // Smooth with three curve families
    auto cubic   = cubicSpline2D(path, 20);
    auto bspline = bSplineUniformCubic(path, 300);
    auto bezier  = bezierPiecewise(path, 30);

    // Evaluate
    Metrics Mc = measure(cubic, obstacles);
    Metrics Mb = measure(bspline, obstacles);
    Metrics Mz = measure(bezier, obstacles);

    // Print
    cout<<"=== Metrics (lower is better) ===\n";
    printMetrics("Cubic Spline", Mc);
    printMetrics("B-Spline",     Mb);
    printMetrics("Bezier",       Mz);

    // Decide winners per metric
    struct Entry{ string name; Metrics M; };
    vector<Entry> E = { {"Cubic Spline",Mc}, {"B-Spline",Mb}, {"Bezier",Mz} };

    auto bestBy = [&](auto get){
        int bi=0;
        for(int i=1;i<(int)E.size();++i) if(get(E[i].M) < get(E[bi].M)) bi=i;
        return bi;
    };

    int wColl = bestBy([](const Metrics&M){ return double(M.collisions); });
    int wLen  = bestBy([](const Metrics&M){ return M.length; });
    int wSm   = bestBy([](const Metrics&M){ return M.smooth; });

    cout<<"\n=== Winners by criterion ===\n";
    cout<<"Collisions : "<<E[wColl].name<<"\n";
    cout<<"Length     : "<<E[wLen].name<<"\n";
    cout<<"Smoothness : "<<E[wSm].name<<"\n";

    // Percentage improvements of winner vs others, per metric
    auto printPct = [&](const string& crit, int widx, auto getter){
        cout<<"\n"<<crit<<" – percentage gain of "<<E[widx].name<<" over others:\n";
        for(int i=0;i<(int)E.size();++i){
            if(i==widx) continue;
            double p = pctBetter(getter(E[widx].M), getter(E[i].M));
            cout<<"  vs "<<E[i].name<<": "<<fixed<<setprecision(1)<<p<<"%\n";
        }
    };
    printPct("Collisions", wColl, [](const Metrics&M){ return double(M.collisions); });
    printPct("Length",     wLen,  [](const Metrics&M){ return M.length; });
    printPct("Smoothness", wSm,   [](const Metrics&M){ return M.smooth; });

    // A simple overall suggestion (rank-sum)
    auto score = [&](const Metrics&M){
        // Normalise by min across methods for each metric (lower is better)
        double cmin=min({Mc.collisions,Mb.collisions,Mz.collisions});
        double lmin=min({Mc.length,Mb.length,Mz.length});
        double smin=min({Mc.smooth,Mb.smooth,Mz.smooth});
        auto nz = [](double v){return (v<1e-9)?1e-9:v;};
        return (M.collisions/nz(cmin)) + (M.length/nz(lmin)) + (M.smooth/nz(smin));
    };
    vector<pair<double,string>> rank = {
        { score(Mc), "Cubic Spline" },
        { score(Mb), "B-Spline"     },
        { score(Mz), "Bezier"       }
    };
    sort(rank.begin(), rank.end());
    cout<<"\n=== Overall ranking (rank-sum; lower is better) ===\n";
    for(int i=0;i<3;i++) cout<<i+1<<") "<<rank[i].second<<"  (score="<<fixed<<setprecision(3)<<rank[i].first<<")\n";

    cout<<"\nNote: obstacles are two squares centred around the midline between Start and Goal, forcing Lazy Theta* to detour.\n";
    cout<<"You can tweak obstacle half-size/offset, grid size, sampling density to stress the splines.\n";
    return 0;
}
