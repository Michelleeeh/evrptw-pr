# Compilacion con g++ (Windows/MinGW, WSL o Linux). Para MSVC, agregar todos
# los .cpp al proyecto; no hay dependencias nuevas.
CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17
SRC      := $(wildcard *.cpp)

alns: $(SRC) *.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC)

# Igual que alns, pero cada candidato descartado por el prefiltro exacto se
# evalua de todos modos y el programa aborta si alguno era factible.
alns_verificar: $(SRC) *.h
	$(CXX) $(CXXFLAGS) -DVERIFICAR_PREFILTRO -o $@ $(SRC)

# Test de equivalencia de Ruta (cambios 2 y 4) contra la implementacion original.
test_equivalencia: tests/test_equivalencia.cpp tests/RutaOrig.cpp Ruta.cpp InstanciaReader.cpp
	$(CXX) $(CXXFLAGS) -o tests/$@ $^
	./tests/test_equivalencia instancias/rc104_21.txt 20000
	./tests/test_equivalencia instancias/c101_21.txt 20000
	./tests/test_equivalencia instancias/r101_21.txt 20000

test_redistribucion: tests/test_redistribucion.cpp Ruta.cpp InstanciaReader.cpp
	$(CXX) $(CXXFLAGS) -o tests/$@ $^
	./tests/test_redistribucion instancias/rc104_21.txt
	./tests/test_redistribucion instancias/r104_21.txt

test_politica_carga: tests/test_pol.cpp Ruta.cpp InstanciaReader.cpp
	$(CXX) $(CXXFLAGS) -o tests/$@ $^
	./tests/test_politica_carga instancias/c107_21.txt 3

clean:
	rm -f alns alns_verificar tests/test_equivalencia tests/test_redistribucion
