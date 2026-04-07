#ifndef ROKIDOPENXRANDROIDDEMO_DATABANK_HPP
#define ROKIDOPENXRANDROIDDEMO_DATABANK_HPP

#pragma once

#include<unordered_map>
#include<string>
#include<shared_mutex>
#include<any>
#include<stdexcept>

class DataBank{
public:
    // 禁止复制/移动
    DataBank(const DataBank&)=delete;
    DataBank &operator=(const DataBank&)=delete;

    template<typename T>
    void set(const std::string &id,T &&data){  // 插入或覆盖数据
        std::unique_lock<std::shared_mutex> lock(mMutex);
        mData[id]=std::forward<T>(data);
    }
    bool remove(const std::string &id){  // 删除数据
        std::unique_lock<std::shared_mutex> lock(mMutex);
        return mData.erase(id)>0;
    }
    bool exists(const std::string &id)const{
        std::shared_lock<std::shared_mutex> lock(mMutex);
        return mData.find(id)!=mData.end();
    }
    template<typename T>
    T* get(const std::string &id){ // 获取数据（失败返回 nullptr）
        std::shared_lock<std::shared_mutex> lock(mMutex);
        auto it=mData.find(id);
        if(it==mData.end()) return nullptr;
        return std::any_cast<T>(&it->second);
    }
    template<typename T>
    T& at(const std::string &id){ // 获取数据（失败抛异常）
        std::shared_lock<std::shared_mutex> lock(mMutex);
        auto it=mData.find(id);
        if(it==mData.end()) throw std::runtime_error("DataBank: id not found: "+id);
        return std::any_cast<T &>(it->second);
    }

    static DataBank &Global(){
        static DataBank _inst;
        return _inst;
    }

private:
    DataBank()=default;
    ~DataBank()=default;

private:
    mutable std::shared_mutex mMutex;
    std::unordered_map<std::string,std::any> mData;
};



#endif //ROKIDOPENXRANDROIDDEMO_DATABANK_HPP
