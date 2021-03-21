// LAST UPDATE: 2021.03.20
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

// --NAMESPACES--
using namespace std;

// DESCRIPTION: TODO...
vector<double> sampling_func(double mini, 
                             double maxi, 
                             int snum, 
                             string stype="linear", 
                             bool included=true)
{
  if(snum <= 2)
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
void print(vector<geometry_msgs::Point> vec)
{
  int vsize = vec.size();
  for(int i = 0; i < vsize; i++)
  {
    cout << i << ": (" << vec[i].x << ", " << vec[i].y << ", " << vec[i].z << ")" << endl; 
  }
}

// DESCRIPTION: TODO...
void print(vector<geometry_msgs::Point32> vec)
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