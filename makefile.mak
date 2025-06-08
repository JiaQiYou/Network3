all:
	gcc -o DistanceVector -std=c99 -Wall DistanceVector.c

clean:
	rm -f DistanceVector