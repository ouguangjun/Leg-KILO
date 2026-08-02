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

#ifndef PMC_UTILS_H_
#define PMC_UTILS_H_

#include <set>
#include <string>
#include <vector>

bool fexists(const char *filename);
void usage(char *argv0);

double get_time();
std::string memory_usage();

void validate(bool condition, const std::string &msg);

void indent(int level);
void indent(int level, std::string str);
void print_max_clique(std::vector<int> &max_clique_data);
void print_n_maxcliques(std::set<std::vector<int>> C, int n);

int getdir(std::string dir, std::vector<std::string> &files);

#endif
