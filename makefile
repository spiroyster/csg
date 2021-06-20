CXX=g++ -g -Iinclude

all: test examples

clean:
	rm -r bin
	mkdir -p bin

test: test/main.cpp
	$(CXX) test/main.cpp -o bin/test

example1: examples/example1.cpp
	$(CXX) examples/example1.cpp -o bin/example1

example2: examples/example2.cpp
	$(CXX) examples/example2.cpp -o bin/example2

example3: examples/example3.cpp
	$(CXX) examples/example3.cpp -o bin/example3

examples: example1 example2 example3
