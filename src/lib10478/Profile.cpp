#include "Profile.hpp"
#include "lib10478/lib10478.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "lib10478/controller.hpp"
#include <string>

using namespace lib10478;

Profile::Profile(std::vector<ProfilePoint>& profile, Length dd)
: profile(profile), dd(dd) 
{ 
}

ProfilePoint Profile::getProfilePoint(Length d)
{
    int index = int(d.convert(m) / this->dd.convert(m));

    if (index >= this->profile.size()) index = this->profile.size() - 1;
    if (index < 0) index = 0;

    return profile[index];
}

//finds the nearest point to the given position
ProfilePoint Profile::getProfilePoint(units::V2Position pos){
    int range = int(to_num(0.5_in /this->dd)); //only checks points within 0.5 inches of the previous point
    Length minDist = 100_tile;
    int index;
    for(int i = prev - range; i < prev + range; i++){
        if (i < 0 || i > profile.size()-1) continue;

        const Length dist = pos.distanceTo(profile[i].pose);

        if(dist < minDist){
            minDist = dist;
            index = i;
        }
    }
    //controller::master.set_text(0,0,std::to_string(minDist.convert(in)));
    prev = index;
    return profile[index];
}

Length Profile::getLength(){
    return dd*profile.size();
}