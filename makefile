main: main.cpp slog.cpp slog.h
	g++ -o main slog.cpp main.cpp -std=c++17 -lfmt -g -O3
