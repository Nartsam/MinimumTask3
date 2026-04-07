#include"scenegui.h"
#include"demos/model.h"
#include"demos/utils.h"
#include"utilsmym.hpp"
#include<GLES3/gl3.h>



SceneGui::SceneGui(){
}
SceneGui::~SceneGui(){
}
void SceneGui::render(const glm::mat4 &p,const glm::mat4 &v,int32_t eye){
    render_button(p,v,eye);
    render_text(p,v,eye);
    render_image(p,v,eye);
}
void SceneGui::render_by_id(const std::string &id,const glm::mat4 &p,const glm::mat4 &v,int32_t eye){
    render_button(p,v,eye,id);
    render_text(p,v,eye,id);
    render_image(p,v,eye,id);
}
bool SceneGui::add_button(const SceneGui::ButtonItem &item,const std::string &id){
    if(!check_id_unique(id)) return false; //id 重复
    //由于ImGui的name不能为空,所以当id为空时自动生成一个不重复的字符串作为构造的name参数
    std::string gui_name=id;
    if(gui_name.empty()) gui_name=random_unique_string();
//    infof("Gui Name: %s",gui_name.c_str())
    auto ptr=std::make_shared<Gui>(gui_name);
    ptr->initialize(item.width+15,item.height+20); //为边框留出距离
    ptr->setModel(item.translate_model);
    ptr->setGuiFlags(ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoBackground|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);
    mButtonList.emplace_back(item,ptr,0,id);
    return true;
}
bool SceneGui::add_text(const SceneGui::TextItem &item,const std::string &id){
    if(!check_id_unique(id)) return false; //id 重复
    Text text_render;
    text_render.initialize();
    mTextList.emplace_back(item,text_render,id);
    return true;
}
SceneGui::TextItem* SceneGui::get_text_item(const std::string &id){
    //static TextItem NullTextItem;
    for(auto & i: mTextList) if(std::get<TEXT_ID_INDEX>(i)==id) return &std::get<0>(i);
    return nullptr;
}
SceneGui::ImageItem *SceneGui::get_image_item(const std::string &id){
    for(auto & i: mImageList) if(std::get<IMAGE_ID_INDEX>(i)==id) return &std::get<0>(i);
    return nullptr;
}
SceneGui::ButtonItem *SceneGui::get_button_item(const std::string &id){
    for(auto & i: mButtonList) if(std::get<BUTTON_ID_INDEX>(i)==id) return &std::get<0>(i);
    return nullptr;
}

bool SceneGui::add_image(const SceneGui::ImageItem &item,const std::string &id){
    if(!check_id_unique(id)) return false; //id 重复
    //由于ImGui的name不能为空,所以当id为空时自动生成一个不重复的字符串作为构造的name参数
    std::string gui_name=id;
    if(gui_name.empty()) gui_name=random_unique_string();
    auto ptr=std::make_shared<Gui>(gui_name);
    ptr->initialize(item.image.cols+15,item.image.rows+20); //为边框留出距离
    infof("=======================AddImage: ID = %s, Image Size: %d,%d",id.c_str(),item.image.cols+15,item.image.rows+20);
    ptr->setModel(item.translate_model);
    ptr->setGuiFlags(ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoBackground|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);
    mImageList.emplace_back(item,ptr,MatToTexture(item.image),id);
    return true;
}

void SceneGui::render_button(const glm::mat4 &p,const glm::mat4 &v,int32_t eye,const std::string &render_id){
    for(auto &[item,gui_ptr,texture_id,id]:mButtonList){
        if(!render_id.empty()&&render_id!=id) continue;
        if(!item.visible) continue;
        bool is_intersect=gui_ptr->isIntersectWithLine(mLinePoint,mLineDirection,false);
        gui_ptr->begin();

//        // 调试画鼠标位置
//        ImVec2 mousePos=ImGui::GetIO().MousePos;
//        ImGui::GetForegroundDrawList()->AddCircle(mousePos,10,IM_COL32(255,0,0,255),12,2);
//        ImVec2 winPos=ImGui::GetWindowPos();   // 窗口左上角(全局坐标)
//        ImVec2 cursor=ImGui::GetCursorScreenPos(); // 下一个Item（按钮）左上角的全局坐标
//        ImVec2 btnMin=cursor;
//        ImVec2 btnMax = ImVec2(cursor.x + 100, cursor.y + 50); // 按钮大小
//        infof("Draw Circle at: %.1f, %.1f, Window Pos: (%.1f, %.1f), Button Min: (%.1f, %.1f)",mousePos.x,mousePos.y, winPos.x, winPos.y, btnMin.x, btnMin.y);
//        infof("-------------------------item hover: %d",ImGui::IsItemHovered());


        if(item.image.empty()){
            ImGui::Button(item.text.c_str(),ImVec2(float(item.width),float(item.height)));
        }
        else{
            if(texture_id==0) texture_id=MatToTexture(item.image);//MatToTexture(item.image,texture_id,texture_id==0); //Old
            ImVec2 size((float)(item.image.cols),(float)(item.image.rows)); // 控制显示大小
            ImGui::Image((ImTextureID)(intptr_t)(texture_id),size);
//            infof("ImGui Image Size: (%.f, %.f) Ratio: %.f",size.x,size.y,size.x*1.0f/size.y);
//            ImGui::ImageButton((ImTextureID)(intptr_t)texture_id,ImVec2(float(item.width),float(item.height)));
        }
        if(is_intersect||item.covered){
            ImU32 rect_color=IM_COL32(0,210,255,255);
            if(mLastPressedTimestamp>0) rect_color=IM_COL32(0,255,0,255);
            ImGui::GetForegroundDrawList()->AddRect(ImGui::GetItemRectMin(),ImGui::GetItemRectMax(),rect_color,0.0f,0,2.0f);
            if(CurrentMSecsSinceEpoch()-mLastTriggerTimestamp<100) item.call();
        }

        gui_ptr->end();
        gui_ptr->setModel(glm::scale(item.translate_model,item.scale));
        gui_ptr->render(p,item.use_view?v:no_view_mat(eye));
    }
    if(mLastTriggerTimestamp>0) mLastTriggerTimestamp=0; //为防止重复触发,更新了一轮后就清空,此变量只会在button中用到,不会影响其他
}
void SceneGui::render_text(const glm::mat4 &p,const glm::mat4 &v,int32_t eye,const std::string &render_id){
    static constexpr int TextBufferLength=1024;
    wchar_t text[TextBufferLength]={0};
    for(auto &[item,text_render,id]:mTextList){
        if(!render_id.empty()&&render_id!=id) continue;
        if(!item.visible) continue;
        const auto &list=StringSplit(item.text,'\n');
        for(int i=0;i<(int)list.size();++i){
            swprintf(text,TextBufferLength,L"%s",list[i].c_str());
            text_render.render(p,item.use_view?v:no_view_mat(eye),glm::scale(glm::translate(item.translate_model,glm::vec3(0.f,-item.line_gap*(float)i,0.f)),item.scale),text,(int)wcslen(text),item.color);
        }
    }
}
void SceneGui::render_image(const glm::mat4 &p,const glm::mat4 &v,int32_t eye,const std::string &render_id){
    for(auto &[item,gui_ptr,texture_id,id]:mImageList){
        if(!render_id.empty()&&render_id!=id) continue;
        if(!item.visible) continue;
        gui_ptr->begin();
        int width=item.image.cols,height=item.image.rows;
        ImVec2 size((float)(width),(float)(height)); // 控制显示大小
//        infof("Current Texture ID: %d",texture_id);
        if(texture_id==0) texture_id=MatToTexture(item.image);//MatToTexture(item.image,texture_id,texture_id==0);  //Old
        ImGui::Image((ImTextureID)(intptr_t)(texture_id),size);
//        infof("ImGui Image Size: (%.f, %.f) Ratio: %.f",size.x,size.y,size.x*1.0f/size.y)
//        auto errCode=glGetError(); infof("render error code: %d", errCode);
        gui_ptr->end();
        gui_ptr->setModel(glm::scale(item.translate_model,item.scale));
        gui_ptr->render(p,item.use_view?v:no_view_mat(eye));
    }
}
void SceneGui::ShowToast(const std::string &text,const glm::mat4 &p,const glm::mat4 &v,const glm::mat4 &model){
    static constexpr int TextBufferLength=1024; static wchar_t ToastChar[TextBufferLength]={0};
    static Text ToastRender; static bool IsFirst=true;
    if(IsFirst) ToastRender.initialize(),IsFirst=false;
    swprintf(ToastChar,TextBufferLength,L"%s",text.c_str());
    ToastRender.render(p,v==glm::mat4(1.0f)?no_view_mat(1):v,model,ToastChar,(int)wcslen(ToastChar),{1.0f,1.0f,1.0f});
}

void SceneGui::rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection){
    mLinePoint=linePoint;
    mLineDirection=lineDirection;
}
void SceneGui::inputEvent(int leftright,const ApplicationEvent &event){ //0-1-0
    bool pressed=false; //未按下
    if(GuiOperationTriggerValue==OperationTrigger::Hand){
        if(leftright==HAND_LEFT) return; // *** 这里只处理了右手的手势
//    if(event.controllerEventBit&CONTROLLER_EVENT_BIT_click_trigger){
//        pressed=true;
//    }
        if(event.click_trigger) pressed=true;
    }
    else if(GuiOperationTriggerValue==OperationTrigger::Controller){
        if(CurrentMSecsSinceEpoch()-mControllerButtonLastPressedTimestamp["select"]<50) pressed=true;
    }
    if(!pressed&&mLastPressedTimestamp>0){ //未被按下,且之前处于被按下的状态(点击)
        mLastTriggerTimestamp=CurrentMSecsSinceEpoch();
        mLastPressedTimestamp=0;
        infof("----------------- Clicked: %lld",mLastTriggerTimestamp);
    }
    else if(pressed&&mLastPressedTimestamp<=0){ //被按下,且之前没有被按下
        mLastPressedTimestamp=CurrentMSecsSinceEpoch();
    }
    //不作处理：被按下,且之前也处于被按下的状态 & 未被按下,且之前没有被按下
}

bool SceneGui::check_id_unique(const std::string &id){
    if(id.empty()) return true;
    for(const auto &i:mButtonList) if(std::get<BUTTON_ID_INDEX>(i)==id) return false;
    for(const auto &i:mTextList) if(std::get<TEXT_ID_INDEX>(i)==id) return false;
    for(const auto &i:mImageList) if(std::get<IMAGE_ID_INDEX>(i)==id) return false;
    return true;
}
bool SceneGui::erase_gui_item(const std::string &id){
    if(id.empty()) return false;
    // button
    for(auto it=mButtonList.begin();it!=mButtonList.end();++it){
        if(std::get<BUTTON_ID_INDEX>(*it)!=id) continue;
        mButtonList.erase(it); return true;
    }
    // text
    for(auto it=mTextList.begin();it!=mTextList.end();++it){
        if(std::get<TEXT_ID_INDEX>(*it)!=id) continue;
        mTextList.erase(it); return true;
    }
    // image
    for(auto it=mImageList.begin();it!=mImageList.end();++it){
        if(std::get<IMAGE_ID_INDEX>(*it)!=id) continue;
        mImageList.erase(it); return true;
    }
    return false;
}

glm::mat4 SceneGui::no_view_mat(int32_t eye){
    //eye1 view[3][0] - eye0 view[3][0]  =  -0.063000
    auto res=glm::mat4(1.0f);
    if(eye==0) return res;
    res[3][0]-=0.063;
    return res;
}
std::string SceneGui::random_unique_string(){
    static constexpr long long MOD=100000000,SHIFT=10000;
    return std::to_string(CurrentMSecsSinceEpoch()%MOD*SHIFT+rand());
}
void SceneGui::keypadEvent(const std::string &key_name){
    mControllerButtonLastPressedTimestamp[key_name]=CurrentMSecsSinceEpoch();
}
void SceneGui::SetGuiOperationTrigger(SceneGui::OperationTrigger trigger){
    extern bool OPENXR_PROGRAM_USE_HAND_AIM; //define in openxr_program.cpp
    GuiOperationTriggerValue=trigger;
    if(trigger==SceneGui::OperationTrigger::Hand) OPENXR_PROGRAM_USE_HAND_AIM=true;
    else OPENXR_PROGRAM_USE_HAND_AIM=false;
}
SceneGui::OperationTrigger SceneGui::GuiOperationTrigger(){
    return GuiOperationTriggerValue;
}
bool SceneGui::set_translate_model(const std::string &id,const glm::mat4 &m){
    if(get_text_item(id)){
        auto p=get_text_item(id); p->translate_model=m; return true;
    }
    if(get_image_item(id)){
        auto p=get_image_item(id); p->translate_model=m; return true;
    }
    if(get_button_item(id)){
        auto p=get_button_item(id); p->translate_model=m; return true;
    }
    return false;
}
glm::mat4 SceneGui::get_translate_model(const std::string &id){
    if(get_text_item(id)){
        auto p=get_text_item(id); return p->translate_model;
    }
    if(get_image_item(id)){
        auto p=get_image_item(id); return p->translate_model;
    }
    if(get_button_item(id)){
        auto p=get_button_item(id); return p->translate_model;
    }
    return {};
}
bool SceneGui::set_button_covered(const std::string & id, bool covered){
    if(get_button_item(id)){
        auto p=get_button_item(id); p->covered=covered;
        return true;
    }
    else return false;
}
SceneGui &SceneGui::Global(){
    static SceneGui _inst;
    return _inst;
}


