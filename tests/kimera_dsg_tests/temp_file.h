#pragma once
#include <stdlib.h>
#include <unistd.h>
#include <string>

namespace kimera {

struct TempFile {
  TempFile() {
    char default_path[] = "/tmp/dsgtest.XXXXXX";
    auto fd = mkstemp(default_path);
    if (fd == -1) {
      valid = false;
      perror("mkstemp failed: ");
      return;
    } else {
      valid = true;
    }

    close(fd);

    path = std::string(default_path, sizeof(default_path));
  }

  ~TempFile() {
    if (!valid) {
      return;
    }

    if (remove(path.c_str()) != 0) {
      perror("remove failed: ");
    }
  }

  bool valid;
  std::string path;
};

}  // namespace kimera
