#pragma once

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <filesystem>

namespace Utils {

namespace fs = std::filesystem;

inline std::vector<std::string> StringSplit(const std::string &str,
                                            const std::string &delim) {
  std::vector<std::string> elems;
  boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
  return elems;
}

inline void StringToLower(std::string *str) {
  std::transform(str->begin(), str->end(), str->begin(), ::tolower);
}

inline std::string StringReplace(const std::string &str,
                                 const std::string &old_str,
                                 const std::string &new_str) {
  if (old_str.empty()) {
    return str;
  }
  size_t position = 0;
  std::string mod_str = str;
  while ((position = mod_str.find(old_str, position)) != std::string::npos) {
    mod_str.replace(position, old_str.size(), new_str);
    position += new_str.size();
  }
  return mod_str;
}

inline std::string StringZeroPadding(const int &num, const int &size) {
  std::stringstream ss;
  ss << std::setw(size) << std::setfill('0') << num;
  std::string str;
  ss >> str;

  return str;
}

enum class CopyType { COPY, HARD_LINK, SOFT_LINK };

// Append trailing slash to string if it does not yet end with a slash.
inline std::string EnsureTrailingSlash(const std::string &str) {
  if (str.length() > 0) {
    if (str.back() != '/') {
      return str + "/";
    }
  } else {
    return str + "/";
  }
  return str;
}

// Check whether file name has the file extension (case insensitive).
inline bool HasFileExtension(const std::string &file_name,
                             const std::string &ext) {
  if (ext.empty())
    return false;
  if (ext.at(0) == '.')
    return false;
  std::string ext_lower = ext;
  StringToLower(&ext_lower);
  if (file_name.size() >= ext_lower.size() &&
      file_name.substr(file_name.size() - ext_lower.size(), ext_lower.size()) ==
          ext_lower) {
    return true;
  }
  return false;
}

// Split the path into its root and extension, for example,
// "dir/file.jpg" into "dir/file" and ".jpg".
inline void SplitFileExtension(const std::string &path, std::string *root,
                               std::string *ext) {
  const auto parts = StringSplit(path, ".");
  assert(parts.size() != 0);
  if (parts.size() == 1) {
    *root = parts[0];
    *ext = "";
  } else {
    *root = "";
    for (size_t i = 0; i < parts.size() - 1; ++i) {
      *root += parts[i] + ".";
    }
    *root = root->substr(0, root->length() - 1);
    if (parts.back() == "") {
      *ext = "";
    } else {
      *ext = "." + parts.back();
    }
  }
}

// Copy or link file from source to destination path
inline void FileCopy(const std::string &src_path, const std::string &dst_path,
                     CopyType type = CopyType::COPY) {
  switch (type) {
  case CopyType::COPY:
    fs::copy_file(src_path, dst_path);
    break;
  case CopyType::HARD_LINK:
    fs::create_hard_link(src_path, dst_path);
    break;
  case CopyType::SOFT_LINK:
    fs::create_symlink(src_path, dst_path);
    break;
  }
}

// Check if the path points to an existing directory.
inline bool ExistsFile(const std::string &path) {
  return fs::is_regular_file(path);
}

// Check if the path points to an existing directory.
inline bool ExistsDir(const std::string &path) {
  return fs::is_directory(path);
}

// Check if the path points to an existing file or directory.
inline bool ExistsPath(const std::string &path) { return fs::exists(path); }

// Delete the directory if it exists.
inline void DeleteDirIfExists(const std::string &path) {
  if (ExistsDir(path)) {
    fs::remove_all(path);
  }
}

// Create the directory if it does not exist.
inline void CreateDirIfNotExists(const std::string &path) {
  if (!ExistsDir(path)) {
    fs::create_directory(path);
  }
}

// Extract the base name of a path, e.g., "image.jpg" for "/dir/image.jpg".
inline std::string GetPathBaseName(const std::string &path) {
  const std::vector<std::string> names =
      StringSplit(StringReplace(path, "\\", "/"), "/");
  if (names.size() > 1 && names.back() == "") {
    return names[names.size() - 2];
  } else {
    return names.back();
  }
}

inline std::string GetPathBaseNameNoExt(const std::string &path) {
  std::string base_name = GetPathBaseName(path);
  std::string root, ext;
  SplitFileExtension(base_name, &root, &ext);

  return root;
}

// Get the path of the parent directory for the given path.
inline std::string GetParentDir(const std::string &path) {
  return fs::path(path).parent_path().string();
}

// Get the relative path between from and to. Both the from and to paths must
// exist.
inline std::string GetRelativePath(const std::string &from,
                                   const std::string &to) {
  // This implementation is adapted from:
  // https://stackoverflow.com/questions/10167382
  // A native implementation in fs is only available starting
  using namespace fs;

  path from_path = canonical(path(from));
  path to_path = canonical(path(to));

  // Start at the root path and while they are the same then do nothing then
  // when they first diverge take the entire from path, swap it with '..'
  // segments, and then append the remainder of the to path.
  path::const_iterator from_iter = from_path.begin();
  path::const_iterator to_iter = to_path.begin();

  // Loop through both while they are the same to find nearest common
  // directory
  while (from_iter != from_path.end() && to_iter != to_path.end() &&
         (*to_iter) == (*from_iter)) {
    ++to_iter;
    ++from_iter;
  }

  // Replace from path segments with '..' (from => nearest common directory)
  path rel_path;
  while (from_iter != from_path.end()) {
    rel_path /= "..";
    ++from_iter;
  }

  // Append the remainder of the to path (nearest common directory => to)
  while (to_iter != to_path.end()) {
    rel_path /= *to_iter;
    ++to_iter;
  }

  return rel_path.string();
}

// Join multiple paths into one path.
template <typename... T> inline std::string JoinPaths(T const &... paths) {
  fs::path result;
  int unpack[]{0, (result = result / fs::path(paths), 0)...};
  static_cast<void>(unpack);
  return result.string();
}

// Return list of files in directory.
inline std::vector<std::string> GetFileList(const std::string &path) {
  std::vector<std::string> file_list;
  for (auto it = fs::directory_iterator(path); it != fs::directory_iterator();
       ++it) {
    if (fs::is_regular_file(*it)) {
      const fs::path file_path = *it;
      file_list.push_back(file_path.string());
    }
  }
  std::sort(file_list.begin(), file_list.end());
  return file_list;
}

// Return list of files, recursively in all sub-directories.
inline std::vector<std::string> GetRecursiveFileList(const std::string &path) {
  std::vector<std::string> file_list;
  for (auto it = fs::recursive_directory_iterator(path);
       it != fs::recursive_directory_iterator(); ++it) {
    if (fs::is_regular_file(*it)) {
      const fs::path file_path = *it;
      file_list.push_back(file_path.string());
    }
  }
  std::sort(file_list.begin(), file_list.end());
  return file_list;
}

// Return list of directories, recursively in all sub-directories.
inline std::vector<std::string> GetDirList(const std::string &path) {
  std::vector<std::string> dir_list;
  for (auto it = fs::directory_iterator(path); it != fs::directory_iterator();
       ++it) {
    if (fs::is_directory(*it)) {
      const fs::path dir_path = *it;
      dir_list.push_back(dir_path.string());
    }
  }
  std::sort(dir_list.begin(), dir_list.end());
  return dir_list;
}

// Return list of directories, recursively in all sub-directories.
inline std::vector<std::string> GetRecursiveDirList(const std::string &path) {
  std::vector<std::string> dir_list;
  for (auto it = fs::recursive_directory_iterator(path);
       it != fs::recursive_directory_iterator(); ++it) {
    if (fs::is_directory(*it)) {
      const fs::path dir_path = *it;
      dir_list.push_back(dir_path.string());
    }
  }
  std::sort(dir_list.begin(), dir_list.end());
  return dir_list;
}

template <typename T> inline void readData(std::ifstream &ifs, T &data) {
  char *buffer = new char[sizeof(T)];
  ifs.read(buffer, sizeof(T));
  data = *(reinterpret_cast<T *>(buffer));

  delete buffer;
}

template <typename T> inline void writeData(std::ofstream &ofs, T data) {
  char *pData = reinterpret_cast<char *>(&data);
  ofs.write(pData, sizeof(T));
}

template <typename T> inline T lexical_cast(const std::string &str) {
  T var;
  std::stringstream ss;
  ss.str(str);
  ss >> var;
  return var;
}

template <typename T>
inline std::string VectorToCSV(const std::vector<T> &values) {
  std::string string;
  for (const T value : values) {
    string += std::to_string(value) + ", ";
  }
  return string.substr(0, string.length() - 2);
}

inline bool IsNotWhiteSpace(const int character) {
  return character != ' ' && character != '\n' && character != '\r' &&
         character != '\t';
}

inline void StringLeftTrim(std::string *str) {
  str->erase(str->begin(),
             std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

inline void StringRightTrim(std::string *str) {
  str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
             str->end());
}

inline void StringTrim(std::string *str) {
  StringLeftTrim(str);
  StringRightTrim(str);
}

inline std::vector<double> CSVToVector(const std::string &csv) {
  auto elems = StringSplit(csv, ",;");
  std::vector<double> values;
  values.reserve(elems.size());
  for (auto &elem : elems) {
    StringTrim(&elem);
    if (elem.empty()) {
      continue;
    }
    try {
      values.push_back(std::stold(elem));
    } catch (const std::invalid_argument &) {
      return std::vector<double>(0);
    }
  }
  return values;
}

template <class T> inline T stringToNum(const std::string &s) {
  T num;
  std::stringstream ss;
  ss << s;
  ss >> num;

  return num;
}

inline void CreateRecursiveDirIfNotExists(const std::string &path) {
  if (ExistsDir(path))
    return;

  std::string parent = GetParentDir(path);

  // 如果父目录不存在,递归创建父目录
  if (!ExistsDir(parent))
    CreateRecursiveDirIfNotExists(parent);

  // 创建目录
  fs::create_directory(path);
}

inline void appendSpaceToBits(std::string &in_str, int bits) {
  while (in_str.length() < bits) {
    in_str.append(" ");
  }
}

} // namespace Utils