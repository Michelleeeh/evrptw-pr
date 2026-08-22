// ============================================================================
//  InstanciaReader.cpp
//  Implementacion de la lectura de instancias EVRPTW.
// ============================================================================

#include "InstanciaReader.h"
#include <fstream>
#include <sstream>
#include <cctype>

using namespace std;

bool leerInstancia(string filename, vector<Nodo>& mapa, double& cc, double& cb, double& rc, double& rcons) {
    ifstream file(filename);
    if (!file.is_open()) return false;

    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;

        if (line.find("StringID") != string::npos || line.find("Type") != string::npos ||
            line.find("VEHICLE") != string::npos || line.find("CUSTOMER") != string::npos ||
            line.find("NUMBER") != string::npos) continue;

        stringstream ss(line);
        string first_word;
        ss >> first_word;

        if (first_word == "Q") {
            size_t p1 = line.find('/');
            size_t p2 = line.find('/', p1 + 1);
            if (p1 != string::npos && p2 != string::npos) {
                string num = line.substr(p1 + 1, p2 - p1 - 1);
                cb = stod(num);
            }
            else ss >> cb;
            continue;
        }
        if (first_word == "C") {
            size_t p1 = line.find('/');
            size_t p2 = line.find('/', p1 + 1);
            if (p1 != string::npos && p2 != string::npos) {
                string num = line.substr(p1 + 1, p2 - p1 - 1);
                cc = stod(num);
            }
            else ss >> cc;
            continue;
        }
        if (first_word == "r") {
            size_t p1 = line.find('/');
            size_t p2 = line.find('/', p1 + 1);
            if (p1 != string::npos && p2 != string::npos) {
                string num = line.substr(p1 + 1, p2 - p1 - 1);
                rcons = stod(num);
            }
            else ss >> rcons;
            continue;
        }
        if (first_word == "g") {
            size_t p1 = line.find('/');
            size_t p2 = line.find('/', p1 + 1);
            if (p1 != string::npos && p2 != string::npos) {
                string num = line.substr(p1 + 1, p2 - p1 - 1);
                rc = stod(num);
            }
            else ss >> rc;
            continue;
        }

        string tipo;
        double x, y, demanda, ready, due, service;
        if (ss >> tipo >> x >> y >> demanda >> ready >> due >> service) {
            Nodo n;
            n.tipo = tipo;
            n.x = x; n.y = y;
            n.demanda = demanda;
            n.ready_time = ready;
            n.due_date = due;
            n.service_time = service;
            n.es_estacion = (tipo == "f");
            n.es_deposito = (tipo == "d");
            try {
                if (isalpha(first_word[0])) n.id = stoi(first_word.substr(1));
                else n.id = stoi(first_word);
            }
            catch (...) { n.id = mapa.size(); }
            mapa.push_back(n);
        }
    }

    if (cc <= 0) cc = 200.0;
    if (cb <= 0) cb = 16.0;
    if (rcons <= 0) rcons = 1.0;
    if (rc <= 0) rc = 3.47;

    return !mapa.empty();
}
