
#include "SignalProcessing.h"

using namespace Eigen;

Filter::Filter(unsigned int order_, Eigen::VectorXd init_element_) {
    order=order_;

    //Initialise elements
    for(unsigned int i=0; i<order+1; i++) {
        x.push_back(init_element_);
        y.push_back(init_element_);
    }

    //not ready yet
    initialised = false;
}

Filter::Filter(unsigned int order_, Eigen::VectorXd init_element_, std::vector<double> b_, std::vector<double> a_) {
    order=order_;

    //Initialise elements
    for(unsigned int i=0; i<order+1; i++) {
        x.push_back(init_element_);
        y.push_back(init_element_);
    }

    init(b_, a_);
}

Filter::~Filter() {
    initialised = false;
    //Destroy anything?
}


void Filter::init(std::vector<double> b_, std::vector<double> a_) {
    //Only works on same order of filter
    if(order+1!=a_.size() || order+1!=b_.size())
        return;

    a=a_;
    b=b_;

    std::cout << a[0]<< "," << a[1]<< ","<< a[2] << " " << b[0]<< ","  << b[1]<< ","<< b[2]<< "\n";

    //Ready
    initialised = true;
}


void Filter::initButter2low(double fn) {

    //Only works on order two filter
    if(order!=2)
        return;

    a.clear();
    b.clear();

    const double ita =1.0/ tan(M_PI*fn);
    const double q=sqrt(2.0);

    b.push_back(1.0 / (1.0 + q*ita + ita*ita)); //b[0]
    b.push_back(2.0*b[0]); //b[1]
    b.push_back(b[0]); //b[2]
    a.push_back(1.);//a[0]
    a.push_back(-2.0 * (ita*ita - 1.0) * b[0]); //a[1]
    a.push_back((1.0 - q*ita + ita*ita) * b[0]); //a[2]

    //std::cout << a[0]*1000<< "," << a[1]*1000<< ","<< a[2]*1000 << " " << b[0]*1000<< ","  << b[1]*1000<< ","<< b[2]*1000<< "\n";

    //Ready
    initialised = true;
}


Eigen::VectorXd Filter::filt(Eigen::VectorXd elem) {
    if(initialised) {

        //Shift elements and insert new one
        for(unsigned int k=0; k<order; k++) {
            x[k]=x[k+1];
            y[k]=y[k+1];
        }
        x[order]=elem;

        //Apply filter
        y[order] = VectorXd::Zero(elem.size());
        for(unsigned int i=0; i<order+1; i++) {
            y[order] += b[i] * x[order-i];
        }
        for(unsigned int i=1; i<order+1; i++) {
            y[order] -= a[i] * y[order-i];
        }
        y[order] /= a[0];

        return y[order];
    }
    else {
        return VectorXd::Ones(elem.size())*std::nan("Filter not initialised");
    }
}
