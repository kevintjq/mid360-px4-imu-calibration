#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cwchar>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <unistd.h>

namespace mid360_calibration {
class Terminal {
 public:
  struct Check {
    std::string label;
    double value, target;
    std::string unit;
    int precision = 1;
    bool at_least = true;
    bool passed() const { return std::isfinite(value) && (at_least ? value>=target : value<=target); }
  };
  Terminal() : tty_(isatty(STDOUT_FILENO) &&
      (!std::getenv("TERM") || std::string(std::getenv("TERM"))!="dumb")),
      color_(tty_ && !std::getenv("NO_COLOR")) {}
  ~Terminal() { clear(); }
  void clear() {
    if (lines_) {
      // Resizing can reflow the old card; preserve it rather than erase logs.
      if (columns()==last_columns_) std::cout<<"\033["<<lines_<<"A\033[J";
      lines_=0;
    }
  }
  void message(const std::string &text,bool error=false) {
    clear();
    const bool success=text.find("✓")==0;
    std::cout<<paint(error ? "1;31" : success ? "1;32" : "36")<<text<<reset()<<"\n"<<std::flush;
  }
  void panel(const std::string &title,double progress,const std::vector<std::string> &details,
             const std::vector<Check> &checks={}) {
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool new_stage=title!=last_title_;
    if ((!tty_ || !last_live_) && !new_stage && now-last_print_<2) return;
    last_print_=now;last_title_=title;
    render(title,progress,details,checks,false,false);
  }
  void summary(const std::string &title,const std::vector<std::string> &details,
               const std::vector<Check> &checks={},bool success=true) {
    render(title,1,details,checks,true,success);
  }
 private:
  struct Line { std::string text,color; };
  static std::string value(double v,int precision) {
    if (!std::isfinite(v)) return "未取得";
    std::ostringstream s;s<<std::fixed<<std::setprecision(precision)<<v;return s.str();
  }
  static int columns() {
    struct winsize ws {};
    return ioctl(STDOUT_FILENO,TIOCGWINSZ,&ws)==0 && ws.ws_col ? ws.ws_col : 80;
  }
  static int screenRows() {
    struct winsize ws {};
    return ioctl(STDOUT_FILENO,TIOCGWINSZ,&ws)==0 && ws.ws_row ? ws.ws_row : 24;
  }
  static std::string repeat(const std::string &ch,int count) {
    std::string s;for(int i=0;i<count;++i)s+=ch;return s;
  }
  static size_t next(const std::string &s,size_t offset,std::mbstate_t &state,int &width) {
    wchar_t ch;
    size_t n=std::mbrtowc(&ch,s.data()+offset,s.size()-offset,&state);
    if (n==static_cast<size_t>(-1) || n==static_cast<size_t>(-2) || n==0) {
      state=std::mbstate_t{};width=1;return 1;
    }
    width=std::max(0,::wcwidth(ch));return n;
  }
  static int width(const std::string &s) {
    std::mbstate_t state {};int result=0;
    for(size_t i=0;i<s.size();) { int w=0;i+=next(s,i,state,w);result+=w; }
    return result;
  }
  static std::vector<std::string> wrap(const std::string &s,int limit) {
    std::vector<std::string> result;
    std::mbstate_t state {};std::string line;int used=0;
    for(size_t i=0;i<s.size();) {
      if(s[i]=='\n') { result.push_back(line);line.clear();used=0;++i;continue; }
      int w=0;const auto bytes=next(s,i,state,w);
      if(used+w>limit) { result.push_back(line);line.clear();used=0; }
      line+=s.substr(i,bytes);used+=w;i+=bytes;
    }
    result.push_back(line);return result;
  }
  std::string paint(const std::string &code) const { return color_ ? "\033["+code+"m" : ""; }
  const char *reset() const { return color_ ? "\033[0m" : ""; }
  void render(const std::string &title,double progress,const std::vector<std::string> &details,
              const std::vector<Check> &checks,bool persistent,bool success) {
    clear();
    const int cols=columns(),body=std::max(12,std::min(84,cols-6));
    std::vector<Line> content;
    content.push_back({"MID360 / PX4  ·  外参标定","2"});
    content.push_back({title,persistent ? (success ? "1;32" : "1;31") : "1;36"});
    if(!persistent) {
      const double p=std::max(0.0,std::min(1.0,progress));
      const int cells=std::max(4,std::min(24,body-12));
      const int filled=static_cast<int>(p*cells);
      content.push_back({"进度  "+repeat("━",filled)+repeat("·",cells-filled)+"  "+value(p*100,0)+"%","36"});
    }
    content.push_back({"",""});
    for(size_t i=0;i<details.size();++i)content.push_back({details[i],i==0 ? "1" : "2"});
    if(!checks.empty())content.push_back({"",""});
    std::string pending;
    int label_width=0;
    for(const auto &check:checks)label_width=std::max(label_width,width(check.label));
    for(const auto &check:checks) {
      const bool pass=check.passed();
      const std::string mark=pass ? "✓ " : persistent ? "✗ " : "○ ";
      const auto label=check.label+std::string(label_width-width(check.label)+2,' ');
      content.push_back({mark+label+value(check.value,check.precision)+" / "+
                         value(check.target,check.precision)+" "+check.unit,pass ? "32" : persistent ? "31" : "33"});
      if(!pass) { if(!pending.empty())pending+="、";pending+=check.label; }
    }
    if(!checks.empty() && !persistent) {
      content.push_back({"",""});
      content.push_back({pending.empty() ? "显示条件已达标，正在完成阶段检查" : "尚需达标："+pending,pending.empty() ? "32" : "33"});
    }
    std::vector<Line> wrapped;
    for(const auto &line:content) {
      if(!tty_)wrapped.push_back(line);
      else for(const auto &part:wrap(line.text,body))wrapped.push_back({part,line.color});
    }
    const bool card=tty_ && cols>=30;
    const bool live=card && !persistent && wrapped.size()+2<static_cast<size_t>(screenRows());
    if(card)std::cout<<paint("2")<<"  ╭"<<repeat("─",body+2)<<"╮"<<reset()<<"\n";
    for(const auto &line:wrapped) {
      if(card)std::cout<<paint("2")<<"  │ "<<reset();
      std::cout<<paint(line.color)<<line.text<<reset();
      if(card)std::cout<<std::string(body-width(line.text),' ')<<paint("2")<<" │"<<reset();
      std::cout<<"\n";
    }
    if(card)std::cout<<paint("2")<<"  ╰"<<repeat("─",body+2)<<"╯"<<reset()<<"\n";
    else std::cout<<"\n";
    std::cout<<std::flush;
    lines_=live ? wrapped.size()+2 : 0;
    last_live_=live;
    last_columns_=cols;
  }
  bool tty_,color_;
  bool last_live_=true;
  size_t lines_=0;
  int last_columns_=80;
  double last_print_=-1e9;
  std::string last_title_;
};
}  // namespace mid360_calibration
