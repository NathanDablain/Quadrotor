#include "Sim_Time.h"

double Sim_Time::Time_fp(){
    double result = static_cast<double>(this->Seconds) + (static_cast<double>(this->MicroSeconds)/1e6);
    return result;
}

void Sim_Time::operator+=(Sim_Time const& in){
    this->MicroSeconds += in.MicroSeconds;
    if (this->MicroSeconds >= MICROSECONDS_CAP){
        this->Seconds++;
        this->MicroSeconds-=MICROSECONDS_CAP;
    }
    this->Seconds += in.Seconds;
}

void Sim_Time::operator-=(Sim_Time const& in){
    this->MicroSeconds -= in.MicroSeconds;
    if (this->MicroSeconds < 0){
        this->Seconds--;
        this->MicroSeconds+=MICROSECONDS_CAP;
    }
    this->Seconds -= in.Seconds;
}

Sim_Time Sim_Time::operator+(Sim_Time const& in){
    Sim_Time out;
    out.Seconds = this->Seconds + in.Seconds;
    out.MicroSeconds = this->MicroSeconds + in.MicroSeconds;
    if (out.MicroSeconds >= MICROSECONDS_CAP){
        out.Seconds++;
        out.MicroSeconds-=MICROSECONDS_CAP;
    }

    return out;
}

Sim_Time Sim_Time::operator-(Sim_Time const& in){
    Sim_Time out;
    out.Seconds = this->Seconds - in.Seconds;
    out.MicroSeconds = this->MicroSeconds - in.MicroSeconds;
    if (out.MicroSeconds < 0){
        out.Seconds--;
        out.MicroSeconds+=MICROSECONDS_CAP;
    }

    return out;
}

bool Sim_Time::operator<(Sim_Time const& in){
    if (this->Seconds < in.Seconds){
        return true;
    }
    else if (this->Seconds == in.Seconds){
        if (this->MicroSeconds < in.MicroSeconds){
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool Sim_Time::operator<=(Sim_Time const& in){
    if (this->Seconds < in.Seconds){
        return true;
    }
    else if (this->Seconds == in.Seconds){
        if (this->MicroSeconds <= in.MicroSeconds){
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool Sim_Time::operator>(Sim_Time const& in){
    if (this->Seconds < in.Seconds){
        return false;
    }
    else if (this->Seconds == in.Seconds){
        if (this->MicroSeconds <= in.MicroSeconds){
            return false;
        }
        else {
            return true;
        }
    }
    else {
        return true;
    }
}

bool Sim_Time::operator>=(Sim_Time const& in){
    if (this->Seconds < in.Seconds){
        return false;
    }
    else if (this->Seconds == in.Seconds){
        if (this->MicroSeconds < in.MicroSeconds){
            return false;
        }
        else {
            return true;
        }
    }
    else {
        return true;
    }
}

bool Sim_Time::operator==(Sim_Time const& in){
    if ((this->Seconds == in.Seconds)&&(this->MicroSeconds == in.MicroSeconds)){
        return true;
    }
    else {
        return false;
    }
}