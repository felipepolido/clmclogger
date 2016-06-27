#pragma once

#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <memory>

class MRDPlot {
public:
  void alloc(size_t n_ch, size_t n_pt);
  bool readFromFile(const std::string &name);
  bool writeToFile(const std::string &name);
  int findChannel(const std::string &name) const;
  bool setChannelNameAndUnit(const std::string &n, const std::string &u, size_t idx);
  
  std::vector<float> _data;
  float _freq;

protected:
  std::string _name;
  size_t _n_points;
  size_t _n_channels;
  std::vector<std::string> _channelNames;
  std::vector<std::string> _channelUnits;
};







typedef struct mrdplot_data
{
	const char *filename;
	int total_n_numbers;
	int n_points;
	int n_channels;
	float frequency;
	float *data;
	char **names;
	char **units;
} MRDPLOT_DATA;

/*************************************************************************/

MRDPLOT_DATA *malloc_mrdplot_data( int n_channels, int n_points );
MRDPLOT_DATA *read_mrdplot(const char *filename );
void write_mrdplot_file( MRDPLOT_DATA *d );
void write_mrdplot_file( std::string file_name, int total_n_numbers, int n_channels, int n_points, float frequency,
			float *data, std::vector<std::string> names,std::vector<std::string> units);
int find_channel(const char *name, MRDPLOT_DATA *d );
//char *generate_file_name();
char *generate_file_name(const char *prefix);
std::string generate_file_name(std::string prefix);
char *last_data();

void fwrite_reversed( char *p, int i1, int i2, FILE *stream );
/*************************************************************************/
