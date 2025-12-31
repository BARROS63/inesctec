// - User inputs GPS (lat, lon) for Point A (one cage corner)
// - Builds a local 8 m x 4 m cage in meters with A=(0,0) and B=(8,4)
// - Places 2 square obstacles roughly on the diagonal (inflated by drone radius for clearance)
// - Plans with Lazy Theta* on an occupancy grid
// - Smooths with a true cubic B-spline (de Boor, clamped uniform knots)
// - Exports:
//    1) "path.svg" (visualisation: cage, obstacles, Lazy Theta*, B-spline)
//    2) "mission_plan.waypoints" (Mission Planner / QGC WPL 110 file)
//
// Compile:
//   g++ -std=c++17 -O2 -o planner compare_lazytheta_bspline_cage.cpp
//
// Run:
//   ./planner
//
// Notes:
// - Cage axes assumed: +X = East, +Y = North from point A.
// - Uses equirectangular conversion meters <-> lat/lon (good for 8x4 m).
// - If the B-spline collides (sampled), it falls back to Lazy Theta* path.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>
using namespace std;

struct Pt { double x=0, y=0; };

static constexpr double WGS84_R = 6378137.0;

static inline double deg2rad(double d){ return d*M_PI/180.0; }
static inline double rad2deg(double r){ return r*180.0/M_PI; }

static void local_to_gps(double lat0_deg, double lon0_deg, double x_east_m, double y_north_m,
                         double& lat_deg, double& lon_deg)
{
    double lat0 = deg2rad(lat0_deg);
    double dLat = y_north_m / WGS84_R;
    double dLon = x_east_m / (WGS84_R * cos(lat0));
    lat_deg = lat0_deg + rad2deg(dLat);
    lon_deg = lon0_deg + rad2deg(dLon);
}

struct Grid {
    int W, H;
    double res;              // meters per cell
    vector<uint8_t> occ;     // 0 free, 1 blocked
    Grid(int W, int H, double res): W(W), H(H), res(res), occ(W*H, 0) {}
    bool in(int x,int y) const { return x>=0 && y>=0 && x<W && y<H; }
    uint8_t& at(int x,int y) { return occ[y*W + x]; }
    uint8_t  at(int x,int y) const { return occ[y*W + x]; }
};

struct Obstacle {
    double cx, cy;   // center in meters (local)
    double half;     // half-size in meters (square)
};

static void rasterise_square(Grid& g, const Obstacle& o, double inflate_m)
{
    double half = o.half + inflate_m;
    int x0 = (int)floor((o.cx - half)/g.res);
    int x1 = (int)floor((o.cx + half)/g.res);
    int y0 = (int)floor((o.cy - half)/g.res);
    int y1 = (int)floor((o.cy + half)/g.res);

    x0 = max(0, x0); y0 = max(0, y0);
    x1 = min(g.W-1, x1); y1 = min(g.H-1, y1);

    for(int y=y0; y<=y1; y++)
        for(int x=x0; x<=x1; x++)
            g.at(x,y) = 1;
}

static vector<pair<int,int>> bresenham(int x0,int y0,int x1,int y1)
{
    vector<pair<int,int>> pts;
    int dx = abs(x1-x0), dy = -abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx + dy;
    int x=x0, y=y0;
    while(true){
        pts.push_back({x,y});
        if(x==x1 && y==y1) break;
        int e2 = 2*err;
        if(e2 >= dy){ err += dy; x += sx; }
        if(e2 <= dx){ err += dx; y += sy; }
    }
    return pts;
}

static bool lineOfSight(const Grid& g, int x0,int y0,int x1,int y1)
{
    for(auto &p : bresenham(x0,y0,x1,y1)){
        if(!g.in(p.first,p.second) || g.at(p.first,p.second)) return false;
    }
    return true;
}

struct Node {
    int x,y;
    double g,f;
    int px,py;
    bool operator<(const Node& o) const { return f > o.f; } 
};

static vector<Pt> lazyThetaStar(const Grid& g, int sx,int sy, int gx,int gy)
{
    auto h = [&](int x,int y){ return hypot(double(x-gx), double(y-gy)); };
    static const int d8[8][2] = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};

    vector<vector<double>> G(g.H, vector<double>(g.W, 1e100));
    vector<vector<pair<int,int>>> parent(g.H, vector<pair<int,int>>(g.W, {-1,-1}));

    priority_queue<Node> pq;
    G[sy][sx] = 0.0;
    parent[sy][sx] = {sx,sy};
    pq.push({sx,sy,0.0,h(sx,sy),sx,sy});

    while(!pq.empty()){
        Node cur = pq.top(); pq.pop();
        if(cur.x==gx && cur.y==gy) break;
        if(cur.g > G[cur.y][cur.x]) continue;

        for(auto &d : d8){
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            if(!g.in(nx,ny) || g.at(nx,ny)) continue;

            auto [px,py] = parent[cur.y][cur.x];

            if(lineOfSight(g, px,py, nx,ny)){
                double tg = G[py][px] + hypot(double(nx-px), double(ny-py));
                if(tg < G[ny][nx]){
                    G[ny][nx] = tg;
                    parent[ny][nx] = {px,py};
                    pq.push({nx,ny,tg,tg+h(nx,ny),px,py});
                }
            } else {
                double tg = G[cur.y][cur.x] + hypot(double(nx-cur.x), double(ny-cur.y));
                if(tg < G[ny][nx]){
                    G[ny][nx] = tg;
                    parent[ny][nx] = {cur.x,cur.y};
                    pq.push({nx,ny,tg,tg+h(nx,ny),cur.x,cur.y});
                }
            }
        }
    }

    if(parent[gy][gx].first == -1) return {};

    vector<Pt> path;
    int cx=gx, cy=gy;
    while(!(cx==parent[cy][cx].first && cy==parent[cy][cx].second)){
        path.push_back({double(cx), double(cy)});
        auto p = parent[cy][cx];
        cx = p.first; cy = p.second;
    }
    path.push_back({double(sx), double(sy)});
    reverse(path.begin(), path.end());
    return path;
}

static vector<Pt> densify(const vector<Pt>& in, double step)
{
    if(in.size()<2) return in;
    vector<Pt> out;
    out.push_back(in.front());
    for(size_t i=1;i<in.size();i++){
        Pt a=in[i-1], b=in[i];
        double L = hypot(b.x-a.x, b.y-a.y);
        if(L < 1e-9){ continue; }
        int n = max(1, (int)floor(L/step));
        for(int k=1;k<=n;k++){
            double t = (double)k/(double)n;
            out.push_back({a.x + t*(b.x-a.x), a.y + t*(b.y-a.y)});
        }
    }
    return out;
}

static vector<double> uniformClampedKnot(int m, int p)
{
    // control points count = m, degree = p
    // knot count = m + p + 1
    int K = m + p + 1;
    vector<double> U(K, 0.0);
    for(int i=0;i<=p;i++) U[i]=0.0;
    for(int i=p+1;i<K-p-1;i++){
        U[i] = double(i-p) / double(K - 2*p - 1);
    }
    for(int i=K-p-1;i<K;i++) U[i]=1.0;
    return U;
}

static Pt deBoorCubic(const vector<Pt>& C, const vector<double>& U, double u)
{
    const int p=3;
    int m = (int)U.size()-1;

    if(u >= U[m-p-1]) return C.back();

    int i=p;
    while(i < m-p-1 && !(u>=U[i] && u<U[i+1])) ++i;

    vector<Pt> d(p+1);
    for(int r=0;r<=p;r++) d[r]=C[i-p+r];

    for(int r=1;r<=p;r++){
        for(int j=p;j>=r;--j){
            double denom = U[i+j-r+1] - U[i+j-p];
            double alpha = (denom>1e-12) ? (u - U[i+j-p]) / denom : 0.0;
            d[j].x = (1-alpha)*d[j-1].x + alpha*d[j].x;
            d[j].y = (1-alpha)*d[j-1].y + alpha*d[j].y;
        }
    }
    return d[p];
}

static vector<Pt> bsplineCubic(const vector<Pt>& ctrl, int samples)
{
    if(ctrl.size()<4) return ctrl;
    const int p=3;
    vector<double> U = uniformClampedKnot((int)ctrl.size(), p);
    vector<Pt> out; out.reserve(samples+1);
    for(int i=0;i<=samples;i++){
        double u = double(i)/double(samples);
        out.push_back(deBoorCubic(ctrl, U, u));
    }
    return out;
}

static bool pointInAnyObstacle(const Pt& p, const vector<Obstacle>& obs, double inflate_m)
{
    for(const auto& o: obs){
        double half = o.half + inflate_m;
        if(fabs(p.x - o.cx) <= half && fabs(p.y - o.cy) <= half) return true;
    }
    return false;
}

static bool pathCollides(const vector<Pt>& path_m, const vector<Obstacle>& obs, double inflate_m)
{
    for(const auto& p: path_m){
        if(pointInAnyObstacle(p, obs, inflate_m)) return true;
    }
    return false;
}

// ======== SVG drawing ========
static void writeSVG(const string& filename,
                     double cageL, double cageW,
                     const vector<Obstacle>& obs,
                     double inflate_m,
                     const vector<Pt>& lazy_m,
                     const vector<Pt>& bspline_m)
{
    // SVG coordinates: we’ll map meters directly, but flip Y for nicer view
    // Add margins
    double margin = 0.6;
    double minX = -margin, minY = -margin;
    double maxX = cageL + margin, maxY = cageW + margin;

    int pxW = 900;
    int pxH = int(pxW * ( (maxY-minY) / (maxX-minX) ));

    auto X = [&](double x){ return (x-minX)/(maxX-minX)*pxW; };
    auto Y = [&](double y){ return (maxY-y)/(maxY-minY)*pxH; }; // flip

    ofstream f(filename);
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\""<<pxW<<"\" height=\""<<pxH<<"\">\n";
    f << "<rect x=\""<<X(0)<<"\" y=\""<<Y(cageW)<<"\" width=\""<<(X(cageL)-X(0))<<"\" height=\""<<(Y(0)-Y(cageW))<<"\" "
      << "fill=\"white\" stroke=\"black\" stroke-width=\"2\"/>\n";

    // Obstacles (inflated)
    for(const auto& o: obs){
        double half = o.half + inflate_m;
        double x0 = o.cx - half, y0 = o.cy - half;
        double w = 2*half, h = 2*half;
        f << "<rect x=\""<<X(x0)<<"\" y=\""<<Y(y0+h)<<"\" width=\""<<(X(x0+w)-X(x0))<<"\" height=\""<<(Y(y0)-Y(y0+h))<<"\" "
          << "fill=\"#ffb347\" opacity=\"0.55\" stroke=\"#ff8c00\" stroke-width=\"2\"/>\n";
    }

    auto polyline = [&](const vector<Pt>& p, const string& color, double sw){
        if(p.empty()) return;
        f << "<polyline fill=\"none\" stroke=\""<<color<<"\" stroke-width=\""<<sw<<"\" points=\"";
        for(auto& q: p) f << X(q.x) << "," << Y(q.y) << " ";
        f << "\"/>\n";
    };

    // Paths
    polyline(lazy_m,   "#000000", 3.0); // Lazy Theta*
    polyline(bspline_m,"#1f77b4", 3.5); // B-spline

    // Start/Goal markers
    auto circle = [&](double x,double y,const string& fill){
        f << "<circle cx=\""<<X(x)<<"\" cy=\""<<Y(y)<<"\" r=\"8\" fill=\""<<fill<<"\" stroke=\"black\" stroke-width=\"2\"/>\n";
    };
    circle(0,0,"#2ca02c");
    circle(cageL,cageW,"#d62728");

    // Legend
    f << "<rect x=\"20\" y=\"20\" width=\"250\" height=\"120\" fill=\"white\" opacity=\"0.85\" stroke=\"black\"/>\n";
    f << "<line x1=\"35\" y1=\"45\" x2=\"85\" y2=\"45\" stroke=\"#000\" stroke-width=\"3\"/>\n";
    f << "<text x=\"95\" y=\"50\" font-family=\"Arial\" font-size=\"18\">Lazy Theta*</text>\n";
    f << "<line x1=\"35\" y1=\"75\" x2=\"85\" y2=\"75\" stroke=\"#1f77b4\" stroke-width=\"3.5\"/>\n";
    f << "<text x=\"95\" y=\"80\" font-family=\"Arial\" font-size=\"18\">B-spline</text>\n";
    f << "<circle cx=\"60\" cy=\"105\" r=\"7\" fill=\"#2ca02c\" stroke=\"black\" stroke-width=\"2\"/>\n";
    f << "<text x=\"95\" y=\"110\" font-family=\"Arial\" font-size=\"18\">Start (A)</text>\n";
    f << "<circle cx=\"60\" cy=\"130\" r=\"7\" fill=\"#d62728\" stroke=\"black\" stroke-width=\"2\"/>\n";
    f << "<text x=\"95\" y=\"135\" font-family=\"Arial\" font-size=\"18\">Goal (B)</text>\n";

    f << "</svg>\n";
    f.close();
}

// ======== Mission Planner WPL 110 writer ========
static void writeMissionPlannerWPL(const string& filename,
                                  const vector<pair<double,double>>& latlon_points,
                                  double altitude_m)
{
    // Minimal plan:
    // 0: Home (as a normal waypoint entry)
    // 1: Takeoff
    // 2..N-2: Waypoints
    // N-1: Land
    //
    // Format: QGC WPL 110
    // idx current frame command p1 p2 p3 p4 lat lon alt autocontinue
    //
    // frame 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
    // command 22 = NAV_TAKEOFF, 16 = NAV_WAYPOINT, 21 = NAV_LAND
    ofstream f(filename);
    f << "QGC WPL 110\n";

    if(latlon_points.size() < 2){
        cerr << "[WARN] Not enough points to write WPL.\n";
        return;
    }

    const int frame = 3;
    int idx = 0;

    auto emit = [&](int idx, int current, int frame, int cmd,
                    double p1,double p2,double p3,double p4,
                    double lat,double lon,double alt,int autocont)
    {
        f << idx << " " << current << " " << frame << " " << cmd << " "
          << p1 << " " << p2 << " " << p3 << " " << p4 << " "
          << fixed << setprecision(8) << lat << " " << lon << " "
          << setprecision(2) << alt << " " << autocont << "\n";
    };

    // Home (A)
    emit(idx++, 1, frame, 16, 0,0,0,0, latlon_points.front().first, latlon_points.front().second, altitude_m, 1);

    // Takeoff at A to altitude
    emit(idx++, 0, frame, 22, 0,0,0,0, latlon_points.front().first, latlon_points.front().second, altitude_m, 1);

    // Intermediate waypoints (use NAV_WAYPOINT)
    for(size_t i=1; i+1<latlon_points.size(); i++){
        emit(idx++, 0, frame, 16, 0,0,0,0, latlon_points[i].first, latlon_points[i].second, altitude_m, 1);
    }

    // Land at B
    emit(idx++, 0, frame, 21, 0,0,0,0, latlon_points.back().first, latlon_points.back().second, 0.0, 1);

    f.close();
}

// ======== Metrics: length (meters), smoothness (sum turning angles), collisions ========
struct Metrics { double length=0, smooth=0; int collisions=0; };

static Metrics evaluate(const vector<Pt>& path_m, const vector<Obstacle>& obs, double inflate_m)
{
    Metrics M;
    if(path_m.size()<2) return M;

    for(size_t i=1;i<path_m.size();i++)
        M.length += hypot(path_m[i].x-path_m[i-1].x, path_m[i].y-path_m[i-1].y);

    for(size_t i=1;i+1<path_m.size();i++){
        Pt a{path_m[i].x-path_m[i-1].x, path_m[i].y-path_m[i-1].y};
        Pt b{path_m[i+1].x-path_m[i].x, path_m[i+1].y-path_m[i].y};
        double na=hypot(a.x,a.y), nb=hypot(b.x,b.y);
        if(na>1e-9 && nb>1e-9){
            double c=(a.x*b.x+a.y*b.y)/(na*nb);
            c=max(-1.0,min(1.0,c));
            M.smooth += acos(c);
        }
    }

    for(const auto& p: path_m)
        if(pointInAnyObstacle(p, obs, inflate_m)) M.collisions++;

    return M;
}

static double pct_gain(double baseline, double candidate)
{
    // lower is better; gain = (baseline - candidate)/baseline * 100
    if(baseline <= 1e-12) return 0.0;
    return (baseline - candidate) / baseline * 100.0;
}
void writePathCSV(const string& name, const vector<Pt>& p){
    ofstream f(name);
    f << "x,y\n";
    for(auto& q: p) f << q.x << "," << q.y << "\n";
}

void writeObstaclesCSV(const string& name, const vector<Obstacle>& obs, double inflate){
    ofstream f(name);
    f << "cx,cy,half_real,half_inflated\n";
    for(const auto& o: obs){
        f << o.cx << "," << o.cy << "," << o.half << "," << (o.half + inflate) << "\n";
    }
}


void writeMetaJSON(const string& name, double latA,double lonA,double cageL,double cageW,double alt){
    ofstream f(name);
    f << "{\n";
    f << "  \"latA\": " << fixed << setprecision(8) << latA << ",\n";
    f << "  \"lonA\": " << fixed << setprecision(8) << lonA << ",\n";
    f << "  \"cageL\": " << cageL << ",\n";
    f << "  \"cageW\": " << cageW << ",\n";
    f << "  \"alt\": " << alt  << "\n";
    f << "}\n";
}

// ======== Main ========
int main()
{
    // --- Problem constants ---
    const double CAGE_L = 8.0;  // meters (x/east)
    const double CAGE_W = 4.0;  // meters (y/north)

    const double GRID_RES = 0.10;           // 10 cm per cell
    const double DRONE_RADIUS = 0.55;       // ~1.075 m diameter => radius ~0.5375; rounded for safety
    const double OBST_HALF = 0.30;          // 0.60 m square obstacle (before inflation)
    const double ALT_M = 10.0;              // waypoint altitude (relative)

    // Derived grid dims to cover cage
    int W = (int)ceil(CAGE_L / GRID_RES) + 1;
    int H = (int)ceil(CAGE_W / GRID_RES) + 1;
    Grid g(W, H, GRID_RES);

    // --- Read GPS input for point A ---
    double latA=0, lonA=0;
    cout << "Enter Point A GPS latitude (deg): ";
    cin >> latA;
    cout << "Enter Point A GPS longitude (deg): ";
    cin >> lonA;

    // Local coordinates: A=(0,0), B=(8,4)
    Pt A_m{0,0};
    Pt B_m{CAGE_L, CAGE_W};

    // --- Obstacles along diagonal ---
    // diagonal param t in (0,1): place near 1/3 and 2/3
    Obstacle o1{ (1.0/3.0)*CAGE_L, (1.0/3.0)*CAGE_W, OBST_HALF };
    Obstacle o2{ (2.0/3.0)*CAGE_L, (2.0/3.0)*CAGE_W, OBST_HALF };
    vector<Obstacle> obs = {o1, o2};

    // Rasterise inflated obstacles into grid
    for(const auto& o: obs) rasterise_square(g, o, DRONE_RADIUS);

    // Convert meters to grid index
    auto toCell = [&](const Pt& p)->pair<int,int>{
        int cx = (int)round(p.x / g.res);
        int cy = (int)round(p.y / g.res);
        return {cx, cy};
    };

    auto [sx,sy] = toCell(A_m);
    auto [gx,gy] = toCell(B_m);

    // Sanity: ensure start/goal not in obstacle
    if(g.in(sx,sy) && g.at(sx,sy)) { cerr << "[ERRO] Start is inside obstacle (after inflation). Adjust.\n"; return 1; }
    if(g.in(gx,gy) && g.at(gx,gy)) { cerr << "[ERRO] Goal is inside obstacle (after inflation). Adjust.\n"; return 1; }

    // --- Plan with Lazy Theta* (in grid coords) ---
    auto path_cells = lazyThetaStar(g, sx,sy, gx,gy);
    if(path_cells.empty()){
        cerr << "[ERRO] No path found.\n";
        return 1;
    }

    // Convert path to meters
    vector<Pt> lazy_m;
    lazy_m.reserve(path_cells.size());
    for(const auto& p: path_cells){
        lazy_m.push_back({ p.x * g.res, p.y * g.res });
    }

    // Densify for smoother B-spline control polygon
    auto ctrl = densify(lazy_m, 0.20); // add points every ~20cm along segments

    // B-spline smoothing (de Boor)
    auto bspline_m = bsplineCubic(ctrl, 600);

    // Collision safety check (sampled points)
    if(pathCollides(bspline_m, obs, DRONE_RADIUS)){
        cerr << "[WARN] B-spline collides (sampled). Falling back to Lazy Theta* polyline.\n";
        bspline_m = lazy_m;
    }

    // --- Write SVG visualisation ---
    writeSVG("path.svg", CAGE_L, CAGE_W, obs, DRONE_RADIUS, lazy_m, bspline_m);
    cout << "[OK] path.svg created (open in a browser).\n";

    // --- Build waypoints from B-spline (downsample) ---
    // Keep it simple: choose ~25 points (plus A/B). Mission Planner does not like huge lists.
    int desired = 25;
    vector<Pt> wp_m;
    if((int)bspline_m.size() <= desired){
        wp_m = bspline_m;
    } else {
        wp_m.reserve(desired);
        for(int i=0;i<desired;i++){
            double t = double(i)/double(desired-1);
            int idx = (int)round(t * (bspline_m.size()-1));
            wp_m.push_back(bspline_m[idx]);
        }
    }

    // Convert local waypoint meters to GPS lat/lon
    vector<pair<double,double>> wp_latlon;
    wp_latlon.reserve(wp_m.size());
    for(const auto& p: wp_m){
        double lat, lon;
        local_to_gps(latA, lonA, p.x, p.y, lat, lon);
        wp_latlon.push_back({lat, lon});
    }

    writeMissionPlannerWPL("mission_plan.waypoints", wp_latlon, ALT_M);

    // --- Terminal metrics and “how much better” (B-spline vs Lazy Theta*) ---
    auto Ml = evaluate(lazy_m,    obs, DRONE_RADIUS);
    auto Mb = evaluate(bspline_m, obs, DRONE_RADIUS);

    cout << "\n=== Metrics (lower is better) ===\n";
    cout << "Lazy Theta*:  collisions=" << Ml.collisions
         << "  length=" << fixed << setprecision(3) << Ml.length << " m  smooth=" << Ml.smooth << " rad\n";

    cout << "\n=== Smoothing effect (B-spline applied to Lazy Theta* path) ===\n";
    cout << "Δ Length:     " << fixed << setprecision(3) << (Mb.length - Ml.length) << " m"
     << "  (" << fixed << setprecision(2) << pct_gain(Ml.length, Mb.length) << " % change vs base)\n";
    cout << "Δ Smoothness: " << fixed << setprecision(3) << (Mb.smooth - Ml.smooth) << " rad"
     << "  (" << fixed << setprecision(2) << pct_gain(Ml.smooth, Mb.smooth) << " % change vs base)\n";
    cout << "Collision delta (base - smooth): " << (Ml.collisions - Mb.collisions) << "\n";

    writePathCSV("lazy.csv", lazy_m);
    writePathCSV("bspline.csv", bspline_m);
    writeObstaclesCSV("obstacles.csv", obs, DRONE_RADIUS);
    writeMetaJSON("meta.json", latA, lonA, CAGE_L, CAGE_W, ALT_M);

    system("python3 ./plot_v2.py obstacles.csv lazy.csv bspline.csv meta.json");

    return 0;
}
