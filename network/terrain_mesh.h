// Piecewise-linear terrain; same grid diagonal as terrain_links.py and Gazebo mesh.
#pragma once
#include "ns3/vector.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

class TerrainMesh {
    std::vector<double> xs,ys,zs;
    double dx=0,dy=0;
public:
    void Load(const std::string& path) {
        std::ifstream f(path);if(!f)throw std::runtime_error("Cannot read terrain");
        std::string line;std::getline(f,line);
        std::map<std::pair<double,double>,double> rows;
        std::set<double> xx,yy;
        while(std::getline(f,line)) {
            std::replace(line.begin(),line.end(),',',' ');std::istringstream s(line);
            double x,y,z;
            if(!(s>>x>>y>>z)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))
                throw std::runtime_error("Invalid terrain row");
            if(!rows.emplace(std::make_pair(x,y),z).second)throw std::runtime_error("Duplicate vertex");
            xx.insert(x);yy.insert(y);
        }
        xs.assign(xx.begin(),xx.end());ys.assign(yy.begin(),yy.end());
        if(xs.size()<2||ys.size()<2||rows.size()!=xs.size()*ys.size())
            throw std::runtime_error("Incomplete terrain grid");
        dx=xs[1]-xs[0];dy=ys[1]-ys[0];
        for(size_t i=1;i<xs.size();i++)if(std::abs(xs[i]-xs[i-1]-dx)>1e-7)
            throw std::runtime_error("Nonuniform X grid");
        for(size_t i=1;i<ys.size();i++)if(std::abs(ys[i]-ys[i-1]-dy)>1e-7)
            throw std::runtime_error("Nonuniform Y grid");
        for(double y:ys)for(double x:xs)zs.push_back(rows.at({x,y}));
    }
    double Height(double x,double y) const {
        if(x<xs.front()-1e-8||x>xs.back()+1e-8||y<ys.front()-1e-8||y>ys.back()+1e-8)
            throw std::runtime_error("Position outside terrain");
        x=std::clamp(x,xs.front(),xs.back());y=std::clamp(y,ys.front(),ys.back());
        size_t i=std::min(xs.size()-2,static_cast<size_t>(std::floor((x-xs.front())/dx)));
        size_t j=std::min(ys.size()-2,static_cast<size_t>(std::floor((y-ys.front())/dy)));
        double u=(x-xs[i])/dx,v=(y-ys[j])/dy;
        auto at=[&](size_t a,size_t b){return zs[b*xs.size()+a];};
        double h00=at(i,j),h10=at(i+1,j),h01=at(i,j+1),h11=at(i+1,j+1);
        return u+v<=1 ? h00+u*(h10-h00)+v*(h01-h00)
                      : h11+(1-u)*(h01-h11)+(1-v)*(h10-h11);
    }
    double MinimumClearance(ns3::Vector a,ns3::Vector b) const {
        Height(a.x,a.y);Height(b.x,b.y);
        std::vector<double> ts={0,1};
        auto cut=[&](double p,double q,double level){
            if(std::abs(q-p)>1e-12){double t=(level-p)/(q-p);if(t>0&&t<1)ts.push_back(t);}
        };
        for(double x:xs)cut(a.x,b.x,x);
        for(double y:ys)cut(a.y,b.y,y);
        double ad=(a.x-xs.front())/dx+(a.y-ys.front())/dy;
        double bd=(b.x-xs.front())/dx+(b.y-ys.front())/dy;
        for(size_t k=0;k<xs.size()+ys.size()-1;k++)cut(ad,bd,k);
        double minimum=std::numeric_limits<double>::infinity();
        for(double t:ts) {
            double x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y),z=a.z+t*(b.z-a.z);
            minimum=std::min(minimum,z-Height(x,y));
        }
        return minimum;
    }
};
