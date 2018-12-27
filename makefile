all:
	cd ./program.xmap && make
	cd ./program.chassis && make
clean:
	cd ./program.xmap && make clean
	cd ./program.chassis && make clean
