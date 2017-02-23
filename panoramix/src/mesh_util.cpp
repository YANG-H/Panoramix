#include "pch.hpp"

#include "mesh_util.hpp"

namespace pano {
namespace core {
Mesh<Point3> LoadMeshFromObjFile(const std::string &fname) {
  using VertHandle = Mesh<Point3>::VertHandle;
  using HalfHandle = Mesh<Point3>::HalfHandle;
  using FaceHandle = Mesh<Point3>::FaceHandle;

  Mesh<Point3> mesh;
  std::ifstream ifs(fname);
  if (ifs.is_open()) {
    std::string line;

    std::vector<std::vector<VertHandle>> face2vhs;

    while (std::getline(ifs, line)) {
      if (line.empty()) {
        continue;
      }
      std::istringstream ss(line);
      std::string token;
      ss >> token;
      if (token == "v") {
        Point3 pos;
        ss >> pos[0] >> pos[1] >> pos[2];
        mesh.addVertex(pos);
      } else if (token == "f") {
        std::vector<VertHandle> vhs;
        while (ss >> token) {
          if (token.empty()) {
            continue;
          }
          int vid = -1;
          size_t p = token.find_first_of('/');
          if (p == std::string::npos) {
            vid = std::stoi(token);
          } else {
            vid = std::stoi(token.substr(0, p));
          }
          assert(vid != -1);
          vhs.push_back(VertHandle(vid - 1));
        }
        if (!vhs.empty()) {
          face2vhs.push_back(std::move(vhs));
        }
      }
    }

    // add face
    std::set<std::pair<VertHandle, VertHandle>> vhpairs_inserted;
    while (!face2vhs.empty()) {
      // find a face sharing any edge with inserted face
      int face_id = 0;
      bool should_inverse = false;
      for (; face_id < face2vhs.size(); face_id++) {
        auto &vhs = face2vhs[face_id];
        for (int i = 0; i < vhs.size(); i++) {
          VertHandle vh1 = vhs[i];
          VertHandle vh2 = vhs[(i + 1) % vhs.size()];
          if (Contains(vhpairs_inserted, std::make_pair(vh1, vh2))) {
            // current halfedge is already occupied
            // inverse current face loop
            should_inverse = true;
            break;
          } else if (Contains(vhpairs_inserted, std::make_pair(vh2, vh1))) {
            should_inverse = false;
            break;
          }
        }
      }
      if (face_id == face2vhs.size()) {
        face_id = 0;
      }
      auto &vhs = face2vhs[face_id];
      if (should_inverse) {
        std::reverse(vhs.begin(), vhs.end());
      }
      mesh.addFace(vhs, false);
      for (int i = 0; i < vhs.size(); i++) {
        VertHandle vh1 = vhs[i];
        VertHandle vh2 = vhs[(i + 1) % vhs.size()];
        vhpairs_inserted.emplace(vh1, vh2);
      }
      // remove inserted face
      std::swap(face2vhs[face_id], face2vhs.back());
      face2vhs.pop_back();
    }
  }
  return mesh;
}
}
}
