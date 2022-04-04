#ifndef COMMON_UTILITY_H
#define COMMON_UTILITY_H

// LAST UPDATE: 2022.03.23
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...COMMON FUNCTIONS

// --OUTSOURCE LIBRARIES--
#include <iostream>
#include <vector>
#include <string>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
#include <std_msgs/Float64MultiArray.h>
#include <octomap_msgs/conversions.h>

// --NAMESPACES--
using namespace std;

// --GLOBAL VARIABLES--
#define PI 3.141592653589793
#define INF std::numeric_limits<double>::infinity()
#define INFINT std::numeric_limits<int>::max()
#define FMAX std::numeric_limits<float>::max()
#define FMIN std::numeric_limits<float>::min()

// DESCRIPTION: TODO...
double randdouble(double from, double to)
{
  double f = (double)rand() / RAND_MAX;
  return from + f * (to - from);  
}

// DESCRIPTION: TODO...
vector<double> sampling_func(double mini, 
                             double maxi, 
                             int snum,
                             bool included=true,
                             string stype="linear")
{
  if(snum < 2)
  { 
    vector<double> v;
    if (included)
    {
      v.push_back(maxi);
    }
    else
    {
      v.push_back(mini);
    }
    return v;
  }
  else if(snum == 2)
  { 
    vector<double> v;
    if (included)
    {
      v.push_back(mini);
      v.push_back(maxi);
    }
    return v;
  }
  else
  {
    int i;
    double srise;
    double srate;
    double s;
    vector<double> v;
    
    if (included)
    {
      v.resize(snum);
    }
    else
    {
      v.resize(snum-2);
    }
    
    if(stype == "increase")
    {
      if (included)
      {
        v[0] = mini;
      }
      s = mini;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        srate = 2 * srise * (i+1) / (snum-2);
        s = s + srate;  
        
        if (included)
        {
          v[i+1] = s;
        }
        else
        {
          v[i] = s;
        }
      }
      if (included)
      {
        v[snum-1] = maxi;
      }
    }
    else if(stype == "decrease")
    {
      int border = snum-3;
      if (included)
      {
        v.push_back(mini);
        border = snum-2;
      }
      
      s = maxi;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < border; i++)
      {
        srate = 2 * srise * (i+1) / (snum-2);
        s = s - srate;  
        v[snum-2-i] = s;
      }
      if (included)
      {
        v[snum-1] = maxi;
      }
    }
    else
    {
      if (included)
      {
        v[0] = mini;
      }
      
      s = mini;
      srise = (maxi - mini) / (snum-1);
      for(i = 0; i < snum-2; i++)
      {
        s = s + srise;
        if (included)
        {
          v[i+1] = s;
        }
        else
        {
          v[i] = s;
        }
      }
      if (included)
      {
        v[snum-1] = maxi;
      }
    }
    return v;
  }
}

// DESCRIPTION: TODO...
template <class T>
void subsample(vector<T>& v, int scount, vector<T>& vsub)
{
  int dv;
  int vsize = v.size();
  if(vsize % scount == 0)
  {
    dv = vsize / scount;
  }
  else
  {
    dv = vsize / scount + 1;
  }

  for (int i = 0; i < scount-1; ++i)
  {
    vsub.push_back(v[i*dv]);
  }
  vsub.push_back(v.back());
}

// DESCRIPTION: TODO...
template <class T>
void subsample(vector<T>& v, int scount)
{
  vector<T> vsub;
  int dv;
  int vsize = v.size();
  if(vsize % scount == 0)
  {
    dv = vsize / scount;
  }
  else
  {
    dv = vsize / scount + 1;
  }

  for (int i = 0; i < scount-1; ++i)
  {
    vsub.push_back(v[i*dv]);
  }
  vsub.push_back(v.back());
  v = vsub;
}

// DESCRIPTION: TODO...
int find_min(vector<double>& v)
{
  if (v.size() > 0)
  {
    int min_index = 0;
    double min = v[0];

    for (int i = 0; i < v.size(); ++i)
    {
      if (v[i] < min)
      {
        min = v[i];
        min_index = i;
      }
    }
    return min_index;
  }
  return -1;
}

// DESCRIPTION: TODO...
int find_max(vector<double>& v)
{
  if (v.size() > 0)
  {
    int max_index = 0;
    double maxi = v[0];

    for (int i = 0; i < v.size(); ++i)
    {
      if (v[i] > maxi)
      {
        maxi = v[i];
        max_index = i;
      }
    }
    return max_index;
  }
  return -1;
}

// DESCRIPTION: TODO...
vector<double> sort_indices(vector<double>& v) 
{
  // initialize original index locations
  vector<double> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),[&v](double i1, double i2) {return v[i1] < v[i2];});

  return idx;
}

// DESCRIPTION: TODO...
vector<double> find_limits(vector<geometry_msgs::Point>& archy)
{
  vector<double> v(6);  // [0]: min_x, [1]: max_x, [2]: min_y, [3]: max_y, [4]: min_z, [5]: max_z
  v[0] = archy[0].x;
  v[1] = archy[0].x;
  v[2] = archy[0].y;
  v[3] = archy[0].y;
  v[4] = archy[0].z;
  v[5] = archy[0].z;

  for(int i = 1; i < archy.size(); i++)
  {
    if(v[0] > archy[i].x)
    {
      v[0] = archy[i].x;
    }

    if(v[1] < archy[i].x)
    {
      v[1] = archy[i].x;
    }

    if(v[2] > archy[i].y)
    {
      v[2] = archy[i].y;
    }

    if(v[3] < archy[i].y)
    {
      v[3] = archy[i].y;
    }

    if(v[4] > archy[i].z)
    {
      v[4] = archy[i].z;
    }

    if(v[5] < archy[i].z)
    {
      v[5] = archy[i].z;
    }
  }
  return v;
}

// DESCRIPTION: TODO...
double find_norm(geometry_msgs::Point p)
{
  return ( sqrt( pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2) ) );
}

// DESCRIPTION: TODO...
double find_Euclidean_distance(geometry_msgs::Point p1, tf::Vector3 p2)
{
  return ( sqrt( pow(p1.x - p2.x(), 2) + pow(p1.y - p2.y(), 2) + pow(p1.z - p2.z(), 2) ) );
}

// DESCRIPTION: TODO...
double find_Euclidean_distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return ( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) ) );
}

// DESCRIPTION: TODO...
double find_Euclidean_distance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
  return ( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) ) );
}

// DESCRIPTION: TODO...
vector<double> find_closest_dist(vector<geometry_msgs::Point>& archy, 
                                 geometry_msgs::Point cell_center)
{
  double temp_dist;
  vector<double> v(2);
  v[0] = find_Euclidean_distance(archy[0], cell_center);
  v[1] = 0;
  for(int i = 1; i < archy.size(); i++)
  {
    temp_dist = find_Euclidean_distance(archy[i], cell_center);
    if(v[0] > temp_dist)
    {
      v[0] = temp_dist;
      v[1] = i;
    }
  }
  return v;
}

// DESCRIPTION: TODO...
bool isInBBx(double px, double py, double pz, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  return (px >= min_x) && (px < max_x) && (py >= min_y) && (py < max_y) && (pz >= min_z) && (pz < max_z);
}

// DESCRIPTION: TODO...
bool isInBBx(geometry_msgs::Point po, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  return (po.x >= min_x) && (po.x < max_x) && (po.y >= min_y) && (po.y < max_y) && (po.z >= min_z) && (po.z < max_z);
}

// DESCRIPTION: TODO...
bool isInBBx(tf::Vector3 po, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  return (po.x() >= min_x) && (po.x() < max_x) && (po.y() >= min_y) && (po.y() < max_y) && (po.z() >= min_z) && (po.z() < max_z);
}

// DESCRIPTION: TODO...
bool isInBBx(tf::Vector3 po, tf::Vector3 mini, tf::Vector3 maxi)
{
  return (po.x() >= mini.x()) && (po.x() < maxi.x()) && (po.y() >= mini.y()) && (po.y() < maxi.y()) && (po.z() >= mini.z()) && (po.z() < maxi.z());
}

// DESCRIPTION: TODO...
bool isInBBx(tf::Vector3 po, octomap::point3d mini, octomap::point3d maxi)
{
  return (po.x() >= mini.x()) && (po.x() < maxi.x()) && (po.y() >= mini.y()) && (po.y() < maxi.y()) && (po.z() >= mini.z()) && (po.z() < maxi.z());
}

// DESCRIPTION: TODO...
double normalize(double mini, double maxi, double val)
{
  return (val - mini) / (maxi - mini);
}

// DESCRIPTION: TODO...
void normalize(vector<double>& vals, vector<double>& normalized_vals)
{
  normalized_vals.clear();

  int min_index = find_min(vals);
  int max_index = find_max(vals);

  for (int i = 0; i < vals.size(); ++i)
  {
    normalized_vals.push_back(normalize(vals[min_index], vals[max_index], vals[i]));
  }
}

// DESCRIPTION: TODO...
void normalize(vector<double>& vals)
{
  vector<double> normalized_vals;

  int min_index = find_min(vals);
  int max_index = find_max(vals);

  for (int i = 0; i < vals.size(); ++i)
  {
    normalized_vals.push_back(normalize(vals[min_index], vals[max_index], vals[i]));
  }
  vals = normalized_vals;
}

// DESCRIPTION: TODO...
void print(vector<int>& vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    if(i == vsize-1)
    {
      cout << i << ") " << vec[i] << endl;;
    }
    else
    {
      cout << i << ") " << vec[i] << ", ";
    }   
  }
}

// DESCRIPTION: TODO...
void print(vector<double>& vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    if(i == vsize-1)
    {
      cout << i << ") " << vec[i] << endl;;
    }
    else
    {
      cout << i << ") " << vec[i] << ", ";
    }   
  }
}

// DESCRIPTION: TODO...
void print(vector< vector<int> >& vecvec)
{
  int count = 0;
  int vsize1 = vecvec.size();
  for(int i = 0; i < vsize1; i++)
  {
    int vsize2 = vecvec[i].size();
    for(int j = 0; j < vsize2; j++)
    {
      cout << count << ") " << vecvec[i][j] << endl;
      count++;
    }
  }
}

// DESCRIPTION: TODO...
void print(vector<geometry_msgs::Point>& vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

// DESCRIPTION: TODO...
void print(vector<geometry_msgs::Point32>& vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

// DESCRIPTION: TODO...
void print(geometry_msgs::Point po)
{
  cout << "(" << po.x << ", " << po.y << ", " << po.z << ")" << endl; 
}

// DESCRIPTION: TODO...
void print(geometry_msgs::Point32 po)
{
  cout << "(" << po.x << ", " << po.y << ", " << po.z << ")" << endl; 
}

// DESCRIPTION: TODO...
void print(vector< vector<double> >& vecvec)
{
  int count = 0;
  int vsize1 = vecvec.size();
  for(int i = 0; i < vsize1; i++)
  {
    int vsize2 = vecvec[i].size();
    for(int j = 0; j < vsize2; j++)
    {
      cout << count << ") " << vecvec[i][j] << endl;
      count++;
    }
  }
}

// DESCRIPTION: TODO...
void print(vector< vector<geometry_msgs::Point> >& vecvec)
{
  int count = 0;
  int vsize1 = vecvec.size();
  for(int i = 0; i < vsize1; i++)
  {
    int vsize2 = vecvec[i].size();
    for(int j = 0; j < vsize2; j++)
    {
      cout << count << ": (" << vecvec[i][j].x << ", " << vecvec[i][j].y << ", " << vecvec[i][j].z << ")" << endl;
      count++;
    }
  }
}

// DESCRIPTION: TODO...
void clear_vector(vector< vector<double> >& vecvec)
{
  int vsize1 = vecvec.size();
  for(int i = 0; i < vsize1; i++)
  {
    vecvec.clear();
  }
}

// DESCRIPTION: TODO...
void vec2msg(vector< vector<double> >& vecvec, std_msgs::Float64MultiArray& vecvec_msg)
{
  int width = vecvec[0].size();
  int height = vecvec.size();
  for(int i = 0; i < height; i++)
  {
    for(int j = 0; j < width; j++)
    {
      vecvec_msg.data.push_back(vecvec[i][j]);
    }
  }
}

// DESCRIPTION: TODO...
void vec2msg(vector<double>& vec, std_msgs::Float64MultiArray& vec_msg)
{
  int vec_size = vec.size();
  for(int i = 0; i < vec_size; i++)
  {
    vec_msg.data.push_back(vec[i]);
  }
}

// DESCRIPTION: TODO...
string createFileName()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
  std::string str(buffer);

  return str;
}

#endif