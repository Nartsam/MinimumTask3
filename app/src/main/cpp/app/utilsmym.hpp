#pragma once
#ifndef ROKIDOPENXRANDROIDDEMO_UTILSMYM_HPP
#define ROKIDOPENXRANDROIDDEMO_UTILSMYM_HPP


// 一些工具函数

#include<string>
#include<sstream>
#include"glm/glm.hpp"
#include<opencv2/opencv.hpp>


//============================ 字符串相关 =====================================
inline bool StringStartsWith(const std::string &str,const std::string &starts_with){
    if((str.rfind(starts_with,0)==0)) return true; //start with
    return false;
}
inline std::string StringTrimmed(const std::string &str){ //去除@str两端的空白字符
    if(str.empty()) return str;
    size_t start=str.find_first_not_of(" \t\n\r\f\v"); //第一个非空白字符的位置
    size_t end=str.find_last_not_of(" \t\n\r\f\v"); //最后一个非空白字符的位置
    if(start==std::string::npos) return {}; //如果字符串全是空白字符
    return str.substr(start,end-start+1);
}
inline std::string MakeSdcardPath(const std::string &path) {
    static const std::string SdcardPrefix("/storage/emulated/0/");
    std::string res=path;
    if(!path.empty() && !StringStartsWith(path,SdcardPrefix)) res=SdcardPrefix+res;
    return res;
}
template<typename... Args>
inline std::string Format(const std::string &fmt,Args... args){
    int size=std::snprintf(nullptr,0,fmt.c_str(),args...)+1;  //首次尝试，估算所需缓冲区大小. +1 for '\0'
    if(size<=0){
        errorf("Format: Error during formatting."); return {};
    }
    std::vector<char> buf(size);
    std::snprintf(buf.data(),size,fmt.c_str(),args...);
    return {buf.data()};
}
inline std::vector<std::string> StringSplit(const std::string &str,char sptor,bool skip_empty=false){
    std::vector<std::string> res;
    std::istringstream iss(str); std::string token;	// 接收缓冲区
    while(getline(iss,token,sptor)) if(!(token.empty()&&skip_empty)) res.push_back(token);
    return res;
}
inline std::string GlmMat4_to_String(const glm::mat4 &matrix,char sptor=' ',bool single_line=false){ //将glm::mat4 转换为 std::string
    std::string res;
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            res+=std::to_string(matrix[i][j]);
            if(j<3) res+=sptor;
            else if(i<3) res+=(single_line?sptor:'\n');
        }
    }
    return res;
}
//从字符串构造glm::mat4矩阵，@def表示构造失败时的返回值
inline glm::mat4 GlmMat4_from_String(const std::string &str,char sptor,const glm::mat4 &def=glm::mat4(1.0f)){
    auto list=StringSplit(str,sptor,true);
    if(list.size()!=4*4) return def;
    glm::mat4 res;
    for(int i=0;i<4;++i)
        for(int j=0;j<4;++j) res[i][j]=std::stof(list[i*4+j]);
    return res;
}

//=============================== 系统相关 ========================================

//以 %Y年%m月%d日%H时%M分%S秒 的形式返回当前时间
inline std::string CurrentDateTime(const std::string &fmt="%Y-%m-%d_%H:%M:%S") {
    auto t=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss<<std::put_time(std::localtime(&t),fmt.c_str());
    return ss.str();
}
inline long long CurrentMSecsSinceEpoch() { //获取unix毫秒时间戳
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
    );
    return ms.count();
}
//判断 @dir 是否存在并且是一个目录
inline bool CheckDirExists(const std::string &dir,bool to_sdcard_path=true) {
    if(dir.empty()) return false;
    DIR *dp;
    if((dp = opendir((to_sdcard_path?MakeSdcardPath(dir):dir).c_str())) == nullptr) return false;
    closedir(dp);
    return true;
}
//创建目录 @dir_path,可创建多级目录,返回创建完成后 @dir_path 是否存在
inline bool MakeDir(const std::string &dir_path,bool to_sdcard_path=true) {
    auto dir=dir_path; if(to_sdcard_path) dir = MakeSdcardPath(dir);
    std::string command("mkdir -p \""+dir+"\"");
    int ret = system(command.c_str());
    if (ret){
        std::stringstream ss;
        ss << "MakeDir Error " << ret << ": " << strerror(errno);
        errorf(ss.str().c_str());
        return false;
    }
    infof(("MakeDir Success: "+dir).c_str());
    return true;
}

//=============================== 矩阵转换相关 =====================================
inline glm::mat4 CV_Matx44f_to_GLM_Mat4(const cv::Matx44f &mat) {  // 将 cv::Matx44f 转换为 glm::mat4
    return {mat(0,0),mat(0,1),mat(0,2),mat(0,3),  // 第一行
            mat(1,0),mat(1,1),mat(1,2),mat(1,3),  // 第二行
            mat(2,0),mat(2,1),mat(2,2),mat(2,3),  // 第三行
            mat(3,0),mat(3,1),mat(3,2),mat(3,3)   // 第四行
    };
}
inline cv::Mat GetDistCoeffs(double k1, double k2, double k3, double p1, double p2) { //构造cv::Mat格式的相机外参矩阵
    cv::Mat distCoeffs=(cv::Mat_<double>(1,5) <<k1, k2, p1, p2, k3);
    return distCoeffs;
}
inline cv::Mat GetCameraMatrix(double fx, double fy, double cx, double cy) { //构造cv::Mat格式的相机内参矩阵
    cv::Mat cameraMatrix=(cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    return cameraMatrix;
}
inline void GetGLModelView(const cv::Matx33f &R, const cv::Vec3f &t, float* mModelView, bool cv2gl){ //2025-06-11修改版
    //绕X轴旋转180度，从OpenCV坐标系变换为OpenGL坐标系
    //同时转置，opengl默认的矩阵为列主序
    if (cv2gl){
        mModelView[0] = R(0, 0);
        mModelView[1] = -R(1, 0);
        mModelView[2] = -R(2, 0);
        mModelView[3] = 0.0f;

        mModelView[4] = R(0, 1);
        mModelView[5] = -R(1, 1);
        mModelView[6] = -R(2, 1);
        mModelView[7] = 0.0f;

        mModelView[8] = R(0, 2);
        mModelView[9] = -R(1, 2);
        mModelView[10] = -R(2, 2);
        mModelView[11] = 0.0f;

        mModelView[12] = t(0);
        mModelView[13] = -t(1);
        mModelView[14] = -t(2);
        mModelView[15] = 1.0f;
    }
    else{
//        mModelView[0] = R(0, 0);
//        mModelView[1] = R(1, 0);
//        mModelView[2] = R(2, 0);
//        mModelView[3] = 0.0f;
//
//        mModelView[4] = R(0, 1);
//        mModelView[5] = R(1, 1);
//        mModelView[6] = R(2, 1);
//        mModelView[7] = 0.0f;
//
//        mModelView[8] = R(0, 2);
//        mModelView[9] = R(1, 2);
//        mModelView[10] = R(2, 2);
//        mModelView[11] = 0.0f;
//
//        mModelView[12] = t(0);
//        mModelView[13] = t(1);
//        mModelView[14] = t(2);
//        mModelView[15] = 1.0f;

        mModelView[0] = R(0, 0);
        mModelView[4] = R(1, 0);
        mModelView[8] = R(2, 0);
        mModelView[12] = 0.0f;

        mModelView[1] = R(0, 1);
        mModelView[5] = R(1, 1);
        mModelView[9] = R(2, 1);
        mModelView[13] = 0.0f;

        mModelView[2] = R(0, 2);
        mModelView[6] = R(1, 2);
        mModelView[10] = R(2, 2);
        mModelView[14] = 0.0f;

        mModelView[3] = t(0);
        mModelView[7] = t(1);
        mModelView[11] = t(2);
        mModelView[15] = 1.0f;
    }
}
inline cv::Matx44f FromRT(const cv::Vec3f &rvec, const cv::Vec3f &tvec, bool cv2gl){
    if (isinf(tvec[0]) || isnan(tvec[0]) || isinf(rvec[0]) || isnan(rvec[0]))
        return {1.0f, 0.0f, 0.0f, 0.0f,0.0f, 1.0f, 0.0f, 0.0f,0.0f, 0.0f, 1.0f, 0.0f,0.0f, 0.0f, 0.0f, 1.0f};
    cv::Matx33f R;
    cv::Rodrigues(rvec, R);
    cv::Matx44f m;
    GetGLModelView(R, tvec, m.val, cv2gl);
    return m;
}

inline cv::Matx44f FromR33T(const cv::Matx33f &R, const cv::Vec3f &tvec, bool cv2gl){
    if (isinf(tvec[0]) || isnan(tvec[0]) )
        return {1.0f, 0.0f, 0.0f, 0.0f,0.0f, 1.0f, 0.0f, 0.0f,0.0f, 0.0f, 1.0f, 0.0f,0.0f, 0.0f, 0.0f, 1.0f};
    cv::Matx44f m;
    GetGLModelView(R, tvec, m.val, cv2gl);
    return m;
}
// 提取glm::mat4矩阵不含scale的部分
inline glm::mat4 GetMatrixWithoutScale(const glm::mat4 &matrix){
    // 提取前3x3矩阵
    glm::mat3 m=glm::mat3(matrix);
    // 使用Gram-Schmidt正交化过程
    glm::vec3 x=glm::vec3(m[0]),y=glm::vec3(m[1]),z=glm::vec3(m[2]);
    // 正交化
    x=glm::normalize(x);
    y=glm::normalize(y-glm::dot(y,x)*x);
    z=glm::normalize(z-glm::dot(z,x)*x-glm::dot(z,y)*y);
    // 确保是右手坐标系
    if(glm::dot(glm::cross(x,y),z)<0){
        z=-z;
    }
    // 重建矩阵
    glm::mat4 result(1.0f);
    result[0]=glm::vec4(x,0.0f);
    result[1]=glm::vec4(y,0.0f);
    result[2]=glm::vec4(z,0.0f);
    // 保持平移
    result[3]=matrix[3];
    return result;
}

// glm::mat4 转换为 cv::Mat（4x4，32位浮点数）// 注意 glm 是列主序,这里仅复制未做转置
inline cv::Mat GLM_Mat4_to_CV_Mat(const glm::mat4& glmMat) {
    cv::Mat cvMat(4, 4, CV_32F); // 创建 4x4 浮点矩阵
    for (int col = 0; col < 4; ++col) {  // 将 glm::mat4 的值按列主序复制到 cv::Mat
        for (int row = 0; row < 4; ++row) {
            cvMat.at<float>(row, col) = glmMat[row][col];
        }
    }
    return cvMat;
}
// 将 cv::Mat 转换为 glm::mat4,假设 cv::Mat 是一个 4x4 的浮点矩阵（CV_32F 类型）：// 注意 glm 是列主序,这里仅复制未做转置
inline glm::mat4 CV_Mat_to_GLM_Mat4(const cv::Mat& cvMat) {
    //CV_Assert(cvMat.rows == 4 && cvMat.cols == 4 && cvMat.type() == CV_32F); // 检查尺寸和类型
    glm::mat4 glmMat(1.0f); // 初始化为单位矩阵
    if(cvMat.rows==4||cvMat.cols==4){
        if(cvMat.type() != CV_32F) warnf(std::string("传入的cv::Mat格式为: "+std::to_string(cvMat.type())+", 会尝试按照CV_32F进行转换,也许会导致错误").c_str());
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) { // 注意：cv 是 row-major，glm 是 column-major
                glmMat[row][col] = cvMat.at<float>(row, col); //这里需要原样复制回来，否则结果不正确. 经测试，这种写法才是正确的(2025-06-10)
                //glmMat[col][row] = cvMat.at<float>(row, col); //还是应该做一个转置(2025-06-11)
            }
        }
    }
    else warnf(std::string("传入的cv::Mat尺寸为("+std::to_string(cvMat.rows)+", "+std::to_string(cvMat.cols)+"), 无法转换glm::mat4, 将返回单位矩阵").c_str());
    return glmMat;
}
inline cv::Mat Matx44f_to_Mat(const cv::Matx44f& matx) {
    cv::Mat mat(4, 4, CV_32F);
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) mat.at<float>(row, col) = matx(row, col);
    return mat;
}
inline cv::Matx44f Mat_to_Matx44f(const cv::Mat& mat) {
    //CV_Assert(mat.rows == 4 && mat.cols == 4 && mat.type() == CV_32F);
    cv::Matx44f matx(1.0f, 0.0f, 0.0f, 0.0f,0.0f, 1.0f, 0.0f, 0.0f,0.0f, 0.0f, 1.0f, 0.0f,0.0f, 0.0f, 0.0f, 1.0f);//初始化为单位矩阵
    if(mat.rows==4||mat.cols==4){
        if(mat.type() != CV_32F) warnf(std::string("传入的cv::Mat格式为: "+std::to_string(mat.type())+", 会尝试按照CV_32F进行转换,也许会导致错误").c_str());
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col) matx(row, col) = mat.at<float>(row, col);
    }
    else warnf(std::string("传入的cv::Mat尺寸为("+std::to_string(mat.rows)+", "+std::to_string(mat.cols)+"), 无法转换Matx44f, 将返回单位矩阵").c_str());
    return matx;
}

//=============================== 图像相关 =====================================

inline cv::Mat ConvertBGRImageToRGB(const cv::Mat &image){
    if(image.channels()==1) return image;
    cv::Mat res;
    if(image.channels()==3) cv::cvtColor(image,res,cv::COLOR_BGR2RGB);
    else if(image.channels()==4) cv::cvtColor(image,res,cv::COLOR_BGRA2RGBA);
    else res=image;
    return res;
}
inline GLuint MatToTexture(const cv::Mat &image){
    if(image.empty()) return 0;
    GLint format=GL_RGB;
    if(image.channels()==1) format=GL_LUMINANCE; // OpenGL Core profile下要用 GL_RED
    else if(image.channels()==3) format=GL_RGB;
    else if(image.channels()==4) format=GL_RGBA;
    int width=image.cols,height=image.rows;
    GLuint textureID;
    glGenTextures(1,&textureID); //infof("Create New OpenGL Texture: %d",textureID);
    glBindTexture(GL_TEXTURE_2D,textureID);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // 关键：防止行对齐导致的挤压
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    glTexImage2D(GL_TEXTURE_2D,0,format,width,height,0,format,GL_UNSIGNED_BYTE,image.data);
    glBindTexture(GL_TEXTURE_2D,0);

//    GLint tex_w,tex_h; 调试用，无实际意义
//    glBindTexture(GL_TEXTURE_2D,textureID);
//    glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_WIDTH,&tex_w);
//    glGetTexLevelParameteriv(GL_TEXTURE_2D,0,GL_TEXTURE_HEIGHT,&tex_h);
//    infof("Texture actual size on GPU: %d x %d\n",tex_w,tex_h);
//    int channels=image.channels();
//    size_t expected_step=image.cols*channels;  // 理论上的每行字节数（不考虑对齐）
//    size_t actual_step=image.step;             // OpenCV 实际存的每行字节数
//    if(expected_step!=actual_step) infof("⚠️  Row padding detected! You MUST call glPixelStorei(GL_UNPACK_ALIGNMENT, 1).")
//    else infof("✅ No row padding, alignment is fine.")
    return textureID;
}


inline cv::Mat RokidCameraMatrix,RokidDistCoeffs; //眼镜相机的 内参 和 外参. 这两项在XrOpenCameraPreview的时候被设置

/*
 * 自行设置在OpenXR中获取到相机图像后要执行的回调函数，@key表示为这个函数分配的唯一ID以便后续找到它
 * 函数的三个参数分别为：@Image 相机图像，@CameraMat 相机参数，@TimeStamp 相机时间戳
*/
inline std::map<int,std::function<void(const cv::Mat&,const cv::Matx44f&,uint64_t)>> CameraUpdateCallbackList;
inline std::mutex CameraUpdateCallbackListMutex;  //操作CallbackList前先加锁


#endif //ROKIDOPENXRANDROIDDEMO_UTILSMYM_HPP