/**
 ============================================================================
 Name        : Parallel Maximum Clique (PMC) Library
 Author      : Ryan A. Rossi   (rrossi@purdue.edu)
 Description : A general high-performance parallel framework for computing
               maximum cliques. The library is designed to be fast for large
               sparse graphs.

 Copyright (C) 2012-2013, Ryan A. Rossi, All rights reserved.

 Please cite the following paper if used:
   Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa
   Patwary, A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs
   and Temporal Strong Components, arXiv preprint 1302.6256, 2013.

 See http://ryanrossi.com/pmc for more information.
 ============================================================================
 */

#include "pmc/pmc_debug_utils.h"
#include "pmc/pmc_utils.h"
#include "pmc/pmc_graph.h"
#include "pmc/pmc_headers.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

using namespace pmc;
using namespace std;


void pmc_graph::initialize() {
    max_degree = 0;
    min_degree = 0;
    avg_degree = 0;
    max_core = 0;
    is_gstats = false;
}

pmc_graph::~pmc_graph() {
}

pmc_graph::pmc_graph(const string& filename) {
    initialize();
    fn = filename;
    read_graph(filename);
}

pmc_graph::pmc_graph(bool graph_stats, const string& filename) {
    initialize();
    fn = filename;
    is_gstats = graph_stats;
    read_graph(filename);
}

pmc_graph::pmc_graph(const string& filename, bool make_adj) {
    initialize();
    fn = filename;
    read_graph(filename);
    if (make_adj) create_adj();
}

void pmc_graph::read_graph(const string& filename) {
    fn = filename;
    double sec = get_time();
    string ext = get_file_extension(filename);

    if (ext == "edges" || ext == "eg2" || ext == "txt")
        read_edges(filename);
    else if (ext == "mtx")
        read_mtx(filename);
    else if (ext == "gr")
        read_metis(filename);
    else {
        cout << "Unsupported graph format." <<endl;
        return;
    }
    basic_stats(sec);
}

void pmc_graph::basic_stats(double sec) {
    cout << "Reading time " << get_time() - sec << endl;
    cout << "|V|: " << num_vertices() <<endl;
    cout << "|E|: " << num_edges() <<endl;
    cout << "p: " << density() <<endl;
    cout << "d_max: " << get_max_degree() <<endl;
    cout << "d_avg: " << get_avg_degree() <<endl;
}


void pmc_graph::read_edges(const string& filename) {
    istringstream in_stream;
    string line = "";
    map< int, vector<int> > vert_list;
    int v = 0, u = 0, self_edges = 0;

    ifstream in_check (filename.c_str());
    if (!in_check) { cout << filename << "File not found!" <<endl; return; }

    bool fix_start_idx = true;
    while (!in_check.eof()) {
        getline(in_check,line);
        if (line[0] == '%' || line[0] == '#') continue;
        if (line != "") {
            in_stream.clear();
            in_stream.str(line);
            in_stream >> v >> u;
            if (v == 0 || u == 0) {
                fix_start_idx = false;
                break;
            }
        }
    }
    ifstream in (filename.c_str());
    if (!in) { cout << filename << "File not found!" <<endl; return; }

    while (!in.eof()) {
        getline(in,line);
        if (line[0] == '%' || line[0] == '#') continue;
        if (line != "") {
            in_stream.clear();
            in_stream.str(line);
            in_stream >> v >> u;

            if (fix_start_idx) {
                v--;
                u--;
            }
            if (v == u)  self_edges++;
            else {
                vert_list[v].push_back(u);
                vert_list[u].push_back(v);
            }
        }
    }
    vertices.push_back(edges.size());
    for (int i=0; i < vert_list.size(); i++) {
        edges.insert(edges.end(),vert_list[i].begin(),vert_list[i].end());
        vertices.push_back(edges.size());
    }
    vert_list.clear();
    vertex_degrees();
    cout << "self-loops: " << self_edges <<endl;
}

pmc_graph::pmc_graph(long long nedges, const int *ei, const int *ej, int offset) {
    initialize();
    map< int, vector<int> > vert_list;
    for (long long i = 0; i < nedges; i++) {
        int v = ei[i] - offset;
        int u = ej[i] - offset;
        if ( v > u ) {
            vert_list[v].push_back(u);
            vert_list[u].push_back(v);
        }
    }
    vertices.push_back(edges.size());
    for (int i=0; i < vert_list.size(); i++) {
        edges.insert(edges.end(),vert_list[i].begin(),vert_list[i].end());
        vertices.push_back(edges.size());
    }
    vert_list.clear();
    vertex_degrees();
}

pmc_graph::pmc_graph(const map<int, vector<int>>& v_map) {
  vertices.push_back(edges.size());
  for (const auto& v_pair : v_map) {
    edges.insert(edges.end(), v_pair.second.begin(), v_pair.second.end());
    vertices.push_back(edges.size());
  }
  vertex_degrees();
}

void pmc_graph::read_mtx(const string& filename) {
    float connStrength = std::numeric_limits<float>::lowest();
    istringstream in2;
    string line="";
    map<int,vector<int> > v_map;
    map<int,vector<double> > valueList;
    int col=0, row=0, ridx=0, cidx=0;
    int entry_counter = 0, num_of_entries = 0;
    double value;

    ifstream in (filename.c_str());
    if(!in) {
        cout<<filename<<" not Found!"<<endl;
        return;
    }

    char data[LINE_LENGTH];
    char banner[LINE_LENGTH];
    char mtx[LINE_LENGTH];
    char crd[LINE_LENGTH];
    char data_type[LINE_LENGTH];
    char storage_scheme[LINE_LENGTH];
    char* p;
    bool b_getValue = true;

    getline(in, line);
    strcpy(data, line.c_str());
    if (sscanf(data, "%s %s %s %s %s", banner, mtx, crd, data_type, storage_scheme) != 5) {
        cout << "ERROR: mtx header is missing" << endl;
        return;
    }

    for (p=data_type; *p!='\0'; *p=tolower(*p),p++);

    if (strcmp(data_type, "pattern") == 0)  b_getValue = false;

    getline(in, line);
    while(line.size()>0&&line[0]=='%') getline(in,line);
    in2.str(line);
    in2 >> row >> col >> num_of_entries;

    if(row!=col) {
        cout<<"* ERROR: This is not a square matrix."<<endl;
        return;
    }

    while(!in.eof() && entry_counter<num_of_entries) {
        getline(in,line);
        entry_counter++;

        if(line!="") {
            in2.clear();
            in2.str(line);
            in2 >> ridx >> cidx >> value;
            ridx--;
            cidx--;

            if (ridx < 0 || ridx >= row)  cout << "sym-mtx error: " << ridx << " row " << row << endl;
            if (cidx < 0 || cidx >= col)  cout << "sym-mtx error: " << cidx << " col " << col << endl;
            if (ridx == cidx)  continue;

            if (ridx > cidx) {
                if (b_getValue) {
                    if(value > connStrength) {
                        v_map[ridx].push_back(cidx);
                        v_map[cidx].push_back(ridx);
                        if (is_gstats) {
                            e_v.push_back(ridx);
                            e_u.push_back(cidx);
                        }
                    }
                } else {
                    v_map[ridx].push_back(cidx);
                    v_map[cidx].push_back(ridx);
                    if (is_gstats) {
                        e_v.push_back(ridx);
                        e_u.push_back(cidx);
                    }
                }

                if (b_getValue && value > connStrength) {
                    valueList[ridx].push_back(value);
                    valueList[cidx].push_back(value);
                }
            } else {
                cout << "* WARNING: Found a nonzero in the upper triangular. ";
                break;
            }
        }
    }
    vertices.push_back(edges.size());
    for (int i=0;i < row; i++) {
        edges.insert(edges.end(),v_map[i].begin(),v_map[i].end());
        vertices.push_back(edges.size());
    }
    v_map.clear();
    valueList.clear();
    vertex_degrees();
}

void pmc_graph::create_adj() {
    double sec = get_time();

    int size = num_vertices();
    adj.resize(size);
    for (int i = 0; i < size; i++) {
        adj[i].resize(size);
    }

    for (int i = 0; i < num_vertices(); i++) {
        for (long long j = vertices[i]; j < vertices[i + 1]; j++ )
            adj[i][edges[j]] = true;
    }
    DEBUG_PRINTF("Created adjacency matrix in %i seconds\n", get_time() - sec);
}


void pmc_graph::sum_vertex_degrees() {
    int n = vertices.size() - 1;

    uint64_t sum = 0;
    for (long long v = 0; v < n; v++) {
        degree[v] = vertices[v+1] - vertices[v];
        sum += (degree[v] * degree[v]-1) / 2;
    }
    cout << "sum of degrees: " << sum <<endl;
}

void pmc_graph::vertex_degrees() {
    int n = static_cast<int>(vertices.size()) - 1;
    degree.resize(n);

    // initialize min and max to degree of first vertex
    min_degree = static_cast<int>(vertices[1]) - static_cast<int>(vertices[0]);
    max_degree = static_cast<int>(vertices[1]) - static_cast<int>(vertices[0]);
    for (int v=0; v<n; v++) {
        degree[v] = static_cast<int>(vertices[v+1]) - static_cast<int>(vertices[v]);
        if (max_degree < degree[v])  {
            max_degree = degree[v];
        }
        if (degree[v] < min_degree)  {
            min_degree = degree[v];
        }
    }
    avg_degree = static_cast<double>(edges.size())/ static_cast<double>(n);
    return;
}

// fast update
void pmc_graph::update_degrees() {
    for (long long v = 0; v < num_vertices(); v++)
        degree[v] = vertices[v+1] - vertices[v];
}

void pmc_graph::update_degrees(bool /*flag*/) {

    int p = 0;
    max_degree = vertices[1] - vertices[0];
    for (long long v = 0; v < num_vertices(); v++) {
        degree[v] = vertices[v+1] - vertices[v];
        if (degree[v] > 0) {
            if (max_degree < degree[v])  max_degree = degree[v];
            p++;
        }
    }
    avg_degree = (double)edges.size() / p;
    return;
}


void pmc_graph::update_degrees(bool_vector& pruned, int& mc) {
    max_degree = -1;
    min_degree = std::numeric_limits<int>::max();
    int p = 0;
    for (long long v=0; v < num_vertices(); v++) {
        degree[v] = vertices[v+1] - vertices[v];
        if (degree[v] < mc) {
            if (!pruned[v])  pruned[v] = true;
            p++;
        }
        else {
            if (max_degree < degree[v])  max_degree = degree[v];
            if (degree[v] < min_degree)  min_degree = degree[v];
        }
    }
    avg_degree = (double)edges.size() / p;
    cout << ", pruned: " << p << endl;
}


void pmc_graph::update_kcores(const bool_vector& pruned) {

    long long n, d, i, j, start, num, md;
    long long v, u, w, du, pu, pw, md_end;
    n = vertices.size();
    kcore.resize(n);
    fill(kcore.begin(), kcore.end(), 0);
    vector <int> pos_tmp(n);
    vector <int> order_tmp(n);

    md = 0;
    for(v=1; v<n; v++) {
        if (!pruned[v-1]) {
            kcore[v] = degree[v-1];
            if (kcore[v] > md)  md = kcore[v];
        }
    }

    md_end = md+1;
    vector < int > bin(md_end,0);

    for (v=1; v < n; v++)  bin[kcore[v]]++;

    start = 1;
    for (d=0; d < md_end; d++) {
        num = bin[d];
        bin[d] = start;
        start = start + num;
    }

    for (v=1; v<n; v++) {
        pos_tmp[v] = bin[kcore[v]];
        order_tmp[pos_tmp[v]] = v;
        bin[kcore[v]]++;
    }

    for (d=md; d > 1; d--)  bin[d] = bin[d-1];
    bin[0] = 1;

    for (i = 1; i < n; i++) {
        v=order_tmp[i];
        if (!pruned[v-1]) {
            for (j = vertices[v-1]; j < vertices[v]; j++) {
                if (!pruned[edges[j]]) {
                    u = edges[j] + 1;
                    if (kcore[u] > kcore[v]) {
                        du = kcore[u];   pu = pos_tmp[u];
                        pw = bin[du];  w = order_tmp[pw];
                        if (u != w) {
                            pos_tmp[u] = pw;   order_tmp[pu] = w;
                            pos_tmp[w] = pu;   order_tmp[pw] = u;
                        }
                        bin[du]++;   kcore[u]--;
                    }
                }
            }
        }
    }

    max_core = 0;
    for (v=0; v<n-1; v++) {
        if (!pruned[v]) {
            kcore[v] = kcore[v+1] + 1; // K+1
            order_tmp[v] = order_tmp[v+1]-1;
            if (kcore[v] > max_core)  max_core = kcore[v];
        }
        else kcore[v] = 0;
    }
    DEBUG_PRINTF("[pmc: updated cores]  K: %i\n", max_core);

    bin.clear();
    pos_tmp.clear();
    order_tmp.clear();
}



string pmc_graph::get_file_extension(const string& filename) {
    string::size_type result;
    string fileExtension = "";
    result = filename.rfind('.', filename.size() - 1);
    if(result != string::npos)
        fileExtension = filename.substr(result+1);
    return fileExtension;
}



void pmc_graph::reduce_graph(const bool_vector& pruned) {
    vector<long long> V(vertices.size(),0);
    vector<int> E;
    E.reserve(edges.size());

    int start = 0;
    for (int i = 0; i < num_vertices(); i++) {
        start = E.size();
        if (!pruned[i]) {
            for (long long j = vertices[i]; j < vertices[i + 1]; j++ ) {
                if (!pruned[edges[j]])
                    E.push_back(edges[j]);
            }
        }
        V[i] = start;
        V[i + 1] = E.size();
    }
    vertices = V;
    edges = E;
}


void pmc_graph::reduce_graph(
        vector<long long>& vs,
        vector<int>& es,
        const bool_vector& pruned) {

    int num_vs = vs.size();

    vector<long long> V(num_vs,0);
    vector<int> E;
    E.reserve(es.size());

    int start = 0;
    for (int i = 0; i < num_vs - 1; i++) {
        start = E.size();
        if (!pruned[i]) { //skip these V_local...
            for (long long j = vs[i]; j < vs[i + 1]; j++ ) {
                if (!pruned[es[j]])
                    E.push_back(es[j]);
            }
        }
        V[i] = start;
        V[i + 1] = E.size();
    }
    vs = V;
    es = E;
}


void pmc_graph::bound_stats(int alg) {
    cout << "graph: " << fn <<endl;
    cout << "alg: " << alg <<endl;
    cout << "-------------------------------" <<endl;
    cout << "Graph Stats for Max-Clique:" <<endl;
    cout << "-------------------------------" <<endl;
    cout << "|V|: " << num_vertices() <<endl;
    cout << "|E|: " << num_edges() <<endl;
    cout << "d_max: " << get_max_degree() <<endl;
    cout << "d_avg: " << get_avg_degree() <<endl;
    cout << "p: " << density() <<endl;
}


void pmc_graph::compute_ordering(vector<int>& bound, vector<int>& order) {
    long long n, d, start, num, md;
    long long v, md_end;

    n = bound.size();
    order.reserve(n);
    vector < long long > pos(n);

    md = 0;
    for(v=1; v<n; v++)
        if (bound[v] > md)  md = bound[v];

    md_end = md+1;
    vector < long long > bin(md_end,0);

    for (v=1; v < n; v++) bin[bound[v]]++;

    start = 1;
    for (d=0; d < md_end; d++) {
        num = bin[d];
        bin[d] = start;
        start = start + num;
    }

    for (v=1; v<n; v++) {
        pos[v] = bin[bound[v]];
        order[pos[v]] = v;
        bin[bound[v]]++;
    }

    for (d=md; d > 1; d--)  bin[d] = bin[d-1];
    bin[0] = 1;

    for (v=0; v<n-1; v++) {
        bound[v] = bound[v+1];
        order[v] = order[v+1]-1;
    }
}

void pmc_graph::degree_bucket_sort() {
    degree_bucket_sort(false);
}



// sort neighbors by degree (largest to smallest)
void pmc_graph::degree_bucket_sort(bool desc) {

    int v, u, n, md, md_end, start, d, num;

    vector<int> tmp_edges;
    tmp_edges.reserve(edges.size());

    for (v = 0; v < num_vertices(); v++) {

        n = vertices[v+1] - vertices[v] + 1;
        vector<int> vert(n);
        vector<int> pos(n);
        vector<int> deg(n);

        md = 0;
        for(u=1; u<n; u++) {
            deg[u] = degree[edges[vertices[v] + (u-1)]];
            if (deg[u] > md)
                md = deg[u];
        }

        md_end = md+1;
        vector < int > bin(md_end,0);

        for (u=1; u < n; u++)  bin[deg[u]]++;

        start = 1;
        for (d=0; d < md_end; d++) {
            num = bin[d];
            bin[d] = start;
            start = start + num;
        }

        for (u=1; u<n; u++) {
            pos[u] = bin[deg[u]];
            vert[pos[u]] = edges[vertices[v] + (u-1)];
            bin[deg[u]]++;
        }

        if (desc) {
            // largest to smallest
            tmp_edges.insert(tmp_edges.end(),vert.rbegin(),vert.rend()-1);
        }
        else {
            //from smallest degree to largest
            tmp_edges.insert(tmp_edges.end(),vert.begin()+1,vert.end());
        }
    }

    DEBUG_PRINTF("[pmc: sorting neighbors]  |E| = %i, |E_sorted| = %i\n", edges.size(), tmp_edges.size());
    edges = tmp_edges;
}



// note when reducing graph explicitly, then forced to read in or keep a copy
bool pmc_graph::clique_test(pmc_graph& G, vector<int> C) {
    int u = 0;
    vector<short> ind(G.num_vertices(),0);
    for (size_t i = 0; i < C.size(); i++) ind[C[i]] = 1;


    // ensure each vertex in C has |C|-1 edges between each other
    for (size_t i = 0; i < C.size(); i++) {
        u = C[i];
        int sz = 0;
        for (long long j = G.vertices[u]; j < G.vertices[u+1]; j++)
            if (ind[G.edges[j]])  sz++;

        // check if connected to |C|-1 vertices
        if (sz != C.size()-1)
            return false;
    }
    return true;
}
