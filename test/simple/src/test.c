#include <math.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  int size = atoi(argv[1]);
  double *array = (double *) malloc(sizeof(double) * size);
  double total  = 0;
  for ( int i = 1; i < size; i++ )
  {
    if ( i % 1000001 == 1000000 )
      array[0] = 100000;
    array[i] = array[i-1] + 1;
  }

  for ( int i = 0; i < size; i++ )
  {
    array[i] += 1;
    total += array[i];
  }
}
