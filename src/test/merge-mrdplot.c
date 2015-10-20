/*****************************************************************************/
/*
  mrdplot-merge.c: merge two mrdplot files
*/
/*****************************************************************************/
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "mrdplot/mrdplot.h"

/*****************************************************************************/

int main (int argc, char **argv)
{
  int i, j, index, c_index;
  char *filename1;
  char *filename2;
  MRDPLOT_DATA *mrdplot_d1 = NULL;
  MRDPLOT_DATA *mrdplot_d2 = NULL;
  int merged_n_points = 0;
  int merged_n_channels = 0;
  MRDPLOT_DATA *d;

  if ( argc <= 1 )
    {
      fprintf( stderr, "Need at least one argument: %s d01703\n", argv[0] );
      exit( -1 );
    }
  filename1 = argv[1];
  if ( argc > 2 )
    filename2 = argv[2];
  else
    filename2 = last_data();

  printf( "Reading: %s\n", filename1 );
  mrdplot_d1 = read_mrdplot( filename1 );

  printf( "Reading: %s\n", filename2 );
  mrdplot_d2 = read_mrdplot( filename2 );

  merged_n_channels = mrdplot_d1->n_channels + mrdplot_d2->n_channels;
  merged_n_points = mrdplot_d1->n_points + mrdplot_d2->n_points;

  d = malloc_mrdplot_data( merged_n_channels, merged_n_points );
  d->filename = generate_file_name();
  if ( mrdplot_d1->frequency != mrdplot_d2->frequency )
    {
      fprintf( stderr, "Frequency mismatch: %g %g\n",
	       mrdplot_d1->frequency, mrdplot_d2->frequency );
      exit( -1 );
    }
  d->frequency = mrdplot_d1->frequency;

  index = 0;
  for ( i = 0; i < mrdplot_d1->n_channels; i++ )
    {
      d->names[index] = mrdplot_d1->names[i];
      d->units[index] = mrdplot_d1->units[i];
      index++;
    }

  for ( i = 0; i < mrdplot_d2->n_channels; i++ )
    {
      d->names[index] = mrdplot_d2->names[i];
      d->units[index] = mrdplot_d2->units[i];
      index++;
    }

  if ( mrdplot_d1->n_points != mrdplot_d2->n_points )
    {
      fprintf( stderr, "n_points mismatch: %d %d\n",
               mrdplot_d1->n_points, mrdplot_d2->n_points );
      exit( -1 );
    }

  for ( i = 0; i < mrdplot_d1->n_points; i++ )
    {
      c_index = 0;
      for ( j = 0; j < mrdplot_d1->n_channels; j++ )
	d->data[i*merged_n_channels + c_index++] = mrdplot_d1->data[i*mrdplot_d1->n_channels + j];
      for ( j = 0; j < mrdplot_d2->n_channels; j++ )
	d->data[i*merged_n_channels + c_index++] = mrdplot_d2->data[i*mrdplot_d2->n_channels + j];
    }

  write_mrdplot_file( d );

  return 0;
}

/********************************************************************/
