#include <iostream>
#include <array>
#include <memory> //for shared_ptr


class LegBase {
public:
    LegBase(std::string name) : name_(name) {};
    const std::string getName() const { return name_; }
private:
    const std::string name_;
};

 
class Joint {
public:
    Joint(LegBase *parent, std::string name) : parent_(parent), name_(name) { }
    const std::string getName() const { return name_; }
    const LegBase *getParent() const { return parent_; }
private:    
    const LegBase *parent_;
    const std::string name_;
};

 

class Link {
public:
    Link(LegBase *parent, std::string name) : parent_(parent), name_(name) { }
    std::string getName() const { return name_; }
    const LegBase *getParent() const { return parent_; }
private:    
    const std::string name_;
    const LegBase *parent_;
};


template <int NJOINTS, int NLINKS>
class Leg : public LegBase {
public:
    Leg(std::string name, std::array<std::shared_ptr<Joint>,NJOINTS> joints, std::array<std::shared_ptr<Link>,NLINKS> links) : LegBase(name), joints_(joints), links_(links) { }
    std::array<std::shared_ptr<Joint>,NJOINTS> joints_;
    std::array<std::shared_ptr<Link>,NLINKS> links_;
};

 
template <int NJOINTS, int NLINKS, int NLEGS>
class Robot { 
public:
    Robot(std::array<std::shared_ptr<Leg<NJOINTS,NLINKS>>,NLEGS> legs) : legs_(legs) {}
    using it = typename std::array<std::shared_ptr<Leg<NJOINTS,NLINKS>>,NLEGS>::iterator;
    it begin() { return it(&legs_[0]); }
    it end() { return it(&legs_[NLEGS]); }

protected:
    std::array<std::shared_ptr<Leg<NJOINTS,NLINKS>>,NLEGS> legs_;

 

public:

 

    template <class Data>
    class LegDataMap : public std::array<Data,NLEGS> { };
    
    template <class Data>
    class LinkDataMap : public std::array<Data,NLEGS*NLINKS> { };
    
    template <class Data>
    class JointDataMap : public std::array<Data,NLEGS*NJOINTS> { };

 

    template <class Data>
    class LegDataMapPair : public std::array<std::pair<std::shared_ptr<Leg<NJOINTS,NLINKS>>, Data*>,NLEGS> {
    public:
        LegDataMapPair(Robot &robot) {
            for (int i=0;i<NLEGS;i++) {
                this->data()[i] = std::make_pair(robot.legs_[i],&data2[i]);
            }
        }
    private:
        Data data2[NLEGS];

 

    };
    
    template <class Data>
    class LinkDataMapPair : public std::array<std::pair<std::shared_ptr<Link>, Data*>,NLINKS*NLEGS> {
    public:
        LinkDataMapPair(Robot &robot) {
            for (int i=0;i<NLEGS;i++) {
                auto leg = robot.legs_[i];
                for (int j=0;j<NLINKS;j++) {
                    auto link = leg->links_[j];
                    this->data()[i*NLINKS+j] = std::make_pair(link,&data2[i*NLINKS+j]);
                }
            }
        }
    private:
        Data data2[NLINKS*NLEGS];
    };

 

    template <class Data>
    class JointDataMapPair : public std::array<std::pair<std::shared_ptr<Joint>, Data*>,NJOINTS*NLEGS> {
    public:
        JointDataMapPair(Robot &robot) {
            for (int i=0;i<NLEGS;i++) {
                auto leg = robot.legs_[i];
                for (int j=0;j<NJOINTS;j++) {
                    auto joint = leg->joints_[j];
                    this->data()[i*NJOINTS+j] = std::make_pair(joint,&data2[i*NJOINTS+j]);
                }
            }
        }
    private:
        Data data2[NJOINTS*NLEGS];
    };    
    
};
 

class HyqLeg : public Leg<3,2> {
public:
    HyqLeg(std::string name) : Leg<3,2>(
        name,
        std::array<std::shared_ptr<Joint>,3>({
            std::make_shared<Joint>(this,"HAA"),
            std::make_shared<Joint>(this,"HFE"),
            std::make_shared<Joint>(this,"KFE")
            }),
        std::array<std::shared_ptr<Link>,2>({
            std::make_shared<Link>(this,"Upper leg"),
            std::make_shared<Link>(this,"Lower leg")
            })
        ) { }
};

 


class Hyq : public Robot<3,2,4> {
public:
    Hyq() :Robot<3,2,4>(std::array<std::shared_ptr<Leg<3,2>>,4>({
        std::make_shared<HyqLeg>("LF"),
        std::make_shared<HyqLeg>("RF"),
        std::make_shared<HyqLeg>("LH"),
        std::make_shared<HyqLeg>("RH"),
        
    })) { 
        std::cout << "-----------------------------------" << std::endl;
        for (int i=0;i<4;i++) {
            std::cout << legs_[i]->getName() << std::endl;
            for (int j=0;j<3;j++)
                std::cout << legs_[i]->joints_[j]->getName() << ",";
            for (int j=0;j<2;j++)
                std::cout << legs_[i]->links_[j]->getName() << ",";
            std::cout << std::endl;
        }
        std::cout << "-----------------------------------" << std::endl;
    
    }
};

 

int main()
{
     int i=0;
    Hyq hyq;
    
    std::cout << "For each leg in robot" << std::endl;
    i=0;
    for (auto l : hyq) {
        std::cout << l->getName() << std::endl;
    }
    
    Hyq::LegDataMap<int> leg_data_map;
    std::cout << "For each value in leg data map" << std::endl;
    for (auto x : leg_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

 

    Hyq::LinkDataMap<int> link_data_map;
    i=0;
    std::cout << "For each value in a link data map" << std::endl;
    for (auto x : link_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }
    
    Hyq::JointDataMap<int> joint_data_map;
    std::cout << "For each value in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

 

    Hyq::LegDataMapPair<int> leg_data_map_pair(hyq);
    std::cout << "For each pair in leg data map" << std::endl;
    i=0;
    for (auto x : leg_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getName() << "=" << *x.second << std::endl;
    }

 

    Hyq::LinkDataMapPair<int> link_data_map_pair(hyq);
    std::cout << "For each pair in link data map" << std::endl;
    i=0;
    for (auto x : link_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << *x.second << std::endl;
    }


    Hyq::JointDataMapPair<int> joint_data_map_pair(hyq);
    std::cout << "For each pair in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << *x.second << std::endl;
    }

 

    return 0;
}