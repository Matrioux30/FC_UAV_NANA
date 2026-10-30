CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

mpc_hw_test: mpc_hw_test.cpp mpc_accel.hpp mpc_formats.h
	$(CXX) $(CXXFLAGS) -o $@ mpc_hw_test.cpp

clean:
	rm -f mpc_hw_test
