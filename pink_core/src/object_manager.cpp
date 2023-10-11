#include "pink_core.hpp"

namespace ps {
    ObjectManager::ObjectManager() {}

    void ObjectManager::init(std::vector<Module*> m) {modules = m;}

    Object* ObjectManager::addObject(const Object& o) {
        auto newObj = new Object(o);
        newObj->id = newUniqueId();
        objects.push_back(newObj);
        for (int i = 0; i < modules.size(); i++)
        {
            modules[i]->registerObject(newObj);
        }
        return newObj;
    }

    void ObjectManager::deleteObject(Object* obj) {
        for (int i = 0; i < modules.size(); i++)
        {
            modules[i]->deregisterObject(obj);
        }
        objects.erase(findObjectInVector(obj->id));
        
        delete obj->rigidbody.shape;
        delete obj->rigidbody.joins;
        delete obj;
    }

    UniqueID ObjectManager::newUniqueId() {
        return ++idCounter;
    }

    Object* ObjectManager::findObject(UniqueID id) {
        Object o;
        o.id = id;
        auto out = std::lower_bound(objects.begin(), objects.end(), &o, ObjectIDCmp());
        return *out;
    }
    std::vector<Object*>::iterator ObjectManager::findObjectInVector(UniqueID id) {
        Object o;
        o.id = id;
        auto out = std::lower_bound(objects.begin(), objects.end(), &o, ObjectIDCmp());
        return out;
    }

    ObjectManager& ObjectManager::GetInstance(){
        static ObjectManager instance;
        return instance;
    }
}