main: main.cpp slog.h
	g++ -o main main.cpp -std=c++17 -lfmt -g -O3 -lpthread
