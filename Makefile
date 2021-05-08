CXXFLAGS += -fPIC -std=c++11 `pkg-config --cflags choreonoid`
CONTROLLER = StanfordController.so

StanfordController.so: StanfordController.o
	g++ -shared -o StanfordController.so StanfordController.o `pkg-config --libs choreonoid` -lCnoidBody
