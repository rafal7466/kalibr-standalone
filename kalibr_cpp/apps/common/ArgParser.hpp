// Tiny command line parser (argparse-like) used by the kalibr_cpp tools.
#pragma once

#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace kalibr {

class ArgParser {
 public:
  struct Opt {
    std::string name, help, defaultValue;
    int nargs;  // 0 = flag, -1 = one or more, n = exactly n
    bool required;
  };

  explicit ArgParser(const std::string& description) : description_(description) {}

  void add(const std::string& name, int nargs, const std::string& help, const std::string& def = "",
           bool required = false) {
    opts_[name] = Opt{name, help, def, nargs, required};
    order_.push_back(name);
  }

  bool parse(int argc, char** argv) {
    prog_ = argv[0];
    if (argc == 1) return false;
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "-h" || a == "--help") return false;
      auto it = opts_.find(a);
      if (it == opts_.end()) throw std::runtime_error("unknown argument: " + a);
      const Opt& o = it->second;
      std::vector<std::string> vals;
      if (o.nargs == 0) {
        vals.push_back("1");
      } else if (o.nargs < 0) {
        while (i + 1 < argc && !isOption(argv[i + 1])) vals.push_back(argv[++i]);
        if (vals.empty()) throw std::runtime_error(a + " expects at least one value");
      } else {
        for (int k = 0; k < o.nargs; ++k) {
          if (i + 1 >= argc) throw std::runtime_error(a + " expects " + std::to_string(o.nargs) + " values");
          vals.push_back(argv[++i]);
        }
      }
      values_[a] = vals;
    }
    for (auto& kv : opts_)
      if (kv.second.required && !values_.count(kv.first))
        throw std::runtime_error("missing required argument " + kv.first);
    return true;
  }

  void printHelp(std::ostream& os = std::cout) const {
    os << description_ << "\n\nUsage: " << prog_ << " [options]\n\nOptions:\n";
    for (auto& n : order_) {
      const Opt& o = opts_.at(n);
      std::string arg = o.nargs == 0 ? "" : (o.nargs < 0 ? " V [V ...]" : (o.nargs == 1 ? " V" : " V1 V2"));
      os << "  " << n << arg << "\n      " << o.help;
      if (!o.defaultValue.empty()) os << " (default: " << o.defaultValue << ")";
      if (o.required) os << " [required]";
      os << "\n";
    }
    if (!epilog.empty()) os << "\n" << epilog << "\n";
  }

  bool has(const std::string& n) const { return values_.count(n) > 0; }
  bool flag(const std::string& n) const { return has(n); }
  std::string str(const std::string& n) const {
    if (has(n)) return values_.at(n)[0];
    return opts_.at(n).defaultValue;
  }
  double dbl(const std::string& n) const { return std::stod(str(n)); }
  int integer(const std::string& n) const { return std::stoi(str(n)); }
  std::vector<std::string> list(const std::string& n) const {
    if (has(n)) return values_.at(n);
    return {};
  }
  std::vector<double> dblList(const std::string& n) const {
    std::vector<double> out;
    for (auto& s : list(n)) out.push_back(std::stod(s));
    return out;
  }

  std::string epilog;

 private:
  static bool isOption(const char* s) {
    return s[0] == '-' && s[1] == '-';
  }
  std::string description_, prog_;
  std::map<std::string, Opt> opts_;
  std::vector<std::string> order_;
  std::map<std::string, std::vector<std::string>> values_;
};

}  // namespace kalibr
