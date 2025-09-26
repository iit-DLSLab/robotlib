
#include "limb.hpp"
#include "iostream"
namespace robotlib
{    
    Limb::Limb( const std::string& name,
                const std::vector<LinkPtr>& links,
                const std::vector<JointPtr>& joints,
                const std::string type)
          : name_(name)
          , joints_(joints)
          , links_(links)
          , type_(type)
          , id(-1)
    {
        // set joint and link sub_id
        if(links.size() < joints.size())
        {
            throw std::runtime_error("Error in Limb constructor: the number of links  is less than the number of joints");
        }
        for(size_t i=0; i<joints_.size(); i++)
        {
            joints_[i]->sub_id = i;
        }
        for(size_t i=0; i<links_.size(); i++)
        {
            links_[i]->sub_id = i;
        }
    };

    Limb::Limb(): name_(""), joints_(), links_(), type_("") {};

    Limb::Limb( const std::string& name,
                const std::vector<std::string>& links_names,
                const std::vector<std::string>& joints_names,
                const std::string type)
          : name_(name), type_(type)
    {
        if(links_names.size() < joints_names.size())
        {
            throw std::runtime_error("Error in Limb constructor: the number of links  is less than the number of joints");
        }

        for(size_t i=0; i<joints_names.size(); i++)
        {
            JointPtr joint = std::make_shared<Joint>(joints_names[i]); // first joint has nullptr as parent link
            joint->sub_id = i;
            joints_.push_back(joint);

            LinkPtr link = std::make_shared<Link>(links_names[i]);
            link->sub_id = i;
            links_.push_back(link);
        }
        if( links_names.size()>joints_names.size() ) // couping with the case where there is a fixed joint
        {
            for(size_t i=joints_names.size(); i<links_names.size(); i++)
            {
                LinkPtr link = std::make_shared<Link>(links_names[i]);//, parent_joint); // all the other links have the last joint as parent joint
                link->sub_id = i;
                links_.push_back(link);
            }
        }
    };

    const std::string& Limb::getName() const 
    { 
        return name_; 
    }

    bool Limb::operator==(const Limb& rhs) const
    {
        if (this->name_ != rhs.name_ || this->id != rhs.id || this->joints_.size() != rhs.joints_.size() || this->links_.size() != rhs.links_.size())
            return false;
        
        for(size_t i = 0; i < this->joints_.size(); i++)
        {
            if (this->joints_[i] != rhs.joints_[i])
                return false;
        }
        for(size_t i = 0; i < this->links_.size(); i++)
        {
            if (this->links_[i] != rhs.links_[i])
                return false;
        }
        return true;
    }
    std::string Limb::type() const
    { 
        return type_; 
    };

    bool Limb::isAttached() const {
        if (id == -1)
            return false;
        else
            return true;
    }

    unsigned int Limb::getNJoints() const { return joints_.size(); };


    unsigned int Limb::getNLinks() const { return links_.size(); };


    const JointPtr Limb::getJoint(const std::string &name) const
    {
        //Iterate over the array of joints to find the joint
        for (auto joint : joints_)
        {
            if (joint->getName().compare(name) == 0)
            {
                return joint;
            }
        }
        throw std::range_error("key not found");
    };


    const LinkPtr Limb::getLink(const std::string &name) const
    {
        //Iterate over the array of links to find the link
        for (auto link : links_)
        {
            if (link->getName().compare(name) == 0)
            {
                return link;
            }
        }
        throw std::range_error("key not found");
    };


    const LinkPtr Limb::getEndEffector() const
    {
        return links_.back();
    };


    const std::vector<JointPtr> Limb::getJoints() const
    { 
        return joints_; 
    }


    std::vector<JointPtr> Limb::getJoints()
    { 
        return joints_; 
    }


    const std::vector<LinkPtr> Limb::getLinks() const
    { 
        return links_;
    };


    std::vector<LinkPtr> Limb::getLinks()
    { 
        return links_;
    };

    void Limb::setType(const std::string& type)
    {
        type_ = type;
    }

} // namespace robotlib