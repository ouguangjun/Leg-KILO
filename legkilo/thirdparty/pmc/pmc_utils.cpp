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

#include "pmc/pmc_utils.h"
#include "pmc/pmc_debug_utils.h"

#include <cassert>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/time.h>

using namespace std;

bool fexists(const char *filename) {
    ifstream ifile(filename);
    return !ifile.fail();
}

void usage(char *argv0) {
    const char *params =
            "Usage: %s -a alg -f graphfile -t threads -o ordering -h heu_strat -u upper_bound -l lower_bound -r reduce_wait_time -w time_limit \n"
            "\t-a algorithm                 : Algorithm for solving MAX-CLIQUE: 0 = full, 1 = no neighborhood cores, 2 = only basic k-core pruning steps  \n"
            "\t-f graph file                : Input GRAPH file for computing the Maximum Clique (matrix market format or simple edge list). \n"
            "\t-o vertex search ordering    : Order in which vertices are searched (default = deg, [kcore, dual_deg, dual_kcore, kcore_deg, rand]) \n"
            "\t-d decreasing order          : Search vertices in DECREASING order. Note if '-d' is not set, then vertices are searched in increasing order by default. \n"
            "\t-e neigh/edge ordering       : Ordering of neighbors/edges (default = deg, [kcore, dual_deg, dual_kcore, kcore_deg, rand]) \n"
            "\t-h heuristic strategy        : Strategy for HEURISTIC method (default = kcore, [deg, dual_deg, dual_kcore, rand, 0 = skip heuristic]) \n"
            "\t-u upper_bound               : UPPER-BOUND on clique size (default = K-cores).\n"
            "\t-l lower_bound               : LOWER-BOUND on clique size (default = Estimate using the Fast Heuristic). \n"
            "\t-t threads                   : Number of THREADS for the algorithm to use (default = 1). \n"
            "\t-r reduce_wait               : Number of SECONDS to wait before inducing the graph based on the unpruned vertices (default = 4 seconds). \n"
            "\t-w time_limit                : Execution TIME LIMIT spent searching for max clique (default = 7 days) \n"
            "\t-k clique size               : Solve K-CLIQUE problem: find clique of size k if it exists. Parameterized to be fast. \n"
            "\t-s stats                     : Compute BOUNDS and other fast graph stats \n"
            "\t-v verbose                   : Output additional details to the screen. \n"
            "\t-? options                   : Print out this help menu. \n";
    fprintf(stderr, params, argv0);
    exit(-1);
}


double get_time() {
    timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec*1.0 + t.tv_usec/1000000.0;
}

string memory_usage() {
    ostringstream mem;
    ifstream proc("/proc/self/status");
    string s;
    while(getline(proc, s), !proc.fail()) {
        if(s.substr(0, 6) == "VmSize") {
            mem << s;
            return mem.str();
        }
    }
    return mem.str();
}

void indent(int level) {
    for (int i = 0; i < level; i++)
        cout << "   ";
    cout << "(" << level << ") ";
}

void print_max_clique(vector<int>& C) {
#ifdef PMC_ENABLE_DEBUG
    cout << "Maximum clique: ";
    for(int i = 0; i < C.size(); i++)
        cout << C[i] + 1 << " ";
    cout << endl;
#else
    discard(C);
#endif
}

void print_n_maxcliques(set< vector<int> > C, int n) {
#ifdef PMC_ENABLE_DEBUG
    set< vector<int> >::iterator it;
    int mc = 0;
    for( it = C.begin(); it != C.end(); it++) {
        if (mc < n) {
            cout << "Maximum clique: ";
            const vector<int>& clq = (*it);
            for (int j = 0; j < clq.size(); j++)
                cout << clq[j] << " ";
            cout <<endl;
            ++mc;
        }
        else break;
    }
#else
    discard(C, n);
#endif
}


void validate(bool condition, const string& msg) {
    if (!condition) {
        cerr << msg << endl;
        assert(condition);
    }
}

int getdir (string dir, vector<string> &files) {
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        if (strcmp(dirp->d_name, ".") != 0)
            files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}


