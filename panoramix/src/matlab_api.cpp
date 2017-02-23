#include "pch.hpp"

#include "matlab_api.hpp"

namespace pano {
namespace misc {

// template helpers
template <typename T> struct RMXType {
  static const mxClassID ClassID = mxUNKNOWN_CLASS;
};
template <int ID> struct RMXID { using type = void; };

#define CONNECT_CLASSID(T, id)                                                 \
  template <> struct RMXType<T> { static const mxClassID ClassID = id; };      \
  template <> struct RMXID<id> { using type = T; };

CONNECT_CLASSID(mxLogical, mxLOGICAL_CLASS)
CONNECT_CLASSID(double, mxDOUBLE_CLASS)
CONNECT_CLASSID(float, mxSINGLE_CLASS)
CONNECT_CLASSID(int8_t, mxINT8_CLASS)
CONNECT_CLASSID(int16_t, mxINT16_CLASS)
CONNECT_CLASSID(int32_t, mxINT32_CLASS)
CONNECT_CLASSID(int64_t, mxINT64_CLASS)
CONNECT_CLASSID(uint8_t, mxUINT8_CLASS)
CONNECT_CLASSID(uint16_t, mxUINT16_CLASS)
CONNECT_CLASSID(uint32_t, mxUINT32_CLASS)
CONNECT_CLASSID(uint64_t, mxUINT64_CLASS)

#undef CONNECT_CLASSID

template <typename T>
inline mxArray *CreateNumericMatrix(const T *d, int m, int n) {
  mxArray *ma = mxCreateNumericMatrix(m, n, RMXType<T>::ClassID, mxREAL);
  std::memcpy(mxGetData(ma), d, sizeof(T) * m * n);
  return ma;
}
template <typename T> inline mxArray *CreateNumericMatrix(int m, int n) {
  return mxCreateNumericMatrix(m, n, RMXType<T>::ClassID, mxREAL);
}

inline mxClassID CVDepthToMxClassID(int cvDepth) {
  switch (cvDepth) {
  case CV_8U:
    return mxUINT8_CLASS;
  case CV_8S:
    return mxINT8_CLASS;
  case CV_16U:
    return mxUINT16_CLASS;
  case CV_16S:
    return mxINT16_CLASS;
  case CV_32S:
    return mxINT32_CLASS;
  case CV_32F:
    return mxSINGLE_CLASS;
  case CV_64F:
    return mxDOUBLE_CLASS;
  default:
    std::cout << "this cv depth type cannot be converted to a matlab class id"
              << std::endl;
    return mxUNKNOWN_CLASS;
  }
}

inline int MxClassIDToCVDepth(mxClassID id) {
  switch (id) {
  case mxLOGICAL_CLASS:
    return CV_8U;
  case mxDOUBLE_CLASS:
    return CV_64F;
  case mxSINGLE_CLASS:
    return CV_32F;
  case mxINT8_CLASS:
    return CV_8S;
  case mxINT16_CLASS:
    return CV_16S;
  case mxINT32_CLASS:
    return CV_32S;
  case mxUINT8_CLASS:
    return CV_8U;
  case mxUINT16_CLASS:
    return CV_16U;
  case mxUINT32_CLASS:
    return CV_32S;
  default:
    std::cout << "this matlab class id cannot be converted to a cv depth type"
              << std::endl;
    return -1;
  }
}

MXA::MXA() : _mxa(0), _destroyWhenOutofScope(false) {}
MXA::MXA(void *mxa, bool dos) : _mxa(mxa), _destroyWhenOutofScope(dos) {}
MXA::MXA(MXA &&a) {
  _mxa = a._mxa;
  a._mxa = 0;
  _destroyWhenOutofScope = a._destroyWhenOutofScope;
  a._destroyWhenOutofScope = false;
}

MXA::MXA(const cv::Mat &im, bool dos /*= false*/)
    : _mxa(0), _destroyWhenOutofScope(false) {
  // collect all dimensions of im
  int channelNum = im.channels();
  mwSize *dimSizes = new mwSize[im.dims + 1];
  for (int i = 0; i < im.dims; i++)
    dimSizes[i] = im.size[i];
  dimSizes[im.dims] = channelNum;

  // create mxArray
  mxArray *ma = mxCreateNumericArray(im.dims + 1, dimSizes,
                                     CVDepthToMxClassID(im.depth()), mxREAL);
  delete[] dimSizes;

  if (!ma)
    return;

  uint8_t *mad = (uint8_t *)mxGetData(ma);
  const size_t szForEachElem = im.elemSize1();
  mwIndex *mxIndices = new mwIndex[im.dims + 1];
  int *cvIndices = new int[im.dims];

  cv::MatConstIterator iter(&im);
  int imTotal = im.total();
  for (int i = 0; i < imTotal; i++, ++iter) {
    // get indices in cv::Mat
    iter.pos(cvIndices);
    // copy indices to mxIndices
    std::copy(cvIndices, cvIndices + im.dims, mxIndices);
    for (mwIndex k = 0; k < channelNum; k++) {
      const uint8_t *fromDataHead = (*iter) + k * szForEachElem;
      mxIndices[im.dims] = k; // set the last indices
      uint8_t *toDataHead =
          mad +
          mxCalcSingleSubscript(ma, im.dims + 1, mxIndices) * szForEachElem;
      std::memcpy(toDataHead, fromDataHead, szForEachElem);
    }
  }

  delete[] mxIndices;
  delete[] cvIndices;

  _mxa = static_cast<void *>(ma);
  _destroyWhenOutofScope = dos;
}

MXA::MXA(double scalar, bool dos /*= false*/)
    : _mxa(0), _destroyWhenOutofScope(false) {
  mwSize dimSizes[] = {1};
  mxArray *ma =
      mxCreateNumericArray(1, dimSizes, mxClassID::mxDOUBLE_CLASS, mxREAL);
  if (!ma)
    return;
  auto d = static_cast<double *>(mxGetData(ma));
  d[0] = scalar;
  _mxa = ma;
  _destroyWhenOutofScope = dos;
}

MXA::MXA(int scalar, bool dos /*= false*/)
    : _mxa(0), _destroyWhenOutofScope(false) {
  mwSize dimSizes[] = {1};
  mxArray *ma =
      mxCreateNumericArray(1, dimSizes, mxClassID::mxDOUBLE_CLASS, mxREAL);
  if (!ma)
    return;
  auto d = static_cast<double *>(mxGetData(ma));
  d[0] = scalar;
  _mxa = ma;
  _destroyWhenOutofScope = dos;
}

MXA::MXA(bool scalar, bool dos /*= false*/)
    : _mxa(0), _destroyWhenOutofScope(false) {
  mwSize dimSizes[] = {1};
  mxArray *ma =
      mxCreateNumericArray(1, dimSizes, mxClassID::mxDOUBLE_CLASS, mxREAL);
  if (!ma)
    return;
  auto d = static_cast<double *>(mxGetData(ma));
  d[0] = scalar;
  _mxa = ma;
  _destroyWhenOutofScope = dos;
}

MXA::MXA(const std::string &string, bool dos /*= false*/)
    : _mxa(0), _destroyWhenOutofScope(false) {
  _mxa = mxCreateStringFromNChars(string.c_str(), string.size());
  _destroyWhenOutofScope = dos;
}

MXA::MXA(const cv::SparseMat &mat, bool dos /*= false*/)
    : _mxa(0), _destroyWhenOutofScope(false) {
  // collect all dimensions of im
  int channelNum = mat.channels();
  assert(channelNum == 1);

  // create mxArray
  int nzc = mat.nzcount();
  mxArray *ma = mxCreateSparse(mat.size(0), mat.size(1), nzc, mxREAL);

  if (!ma)
    return;

  std::vector<std::tuple<int, int, double>> triplets; // col - row - val
  triplets.reserve(nzc);

  THERE_ARE_BOTTLENECKS_HERE(); // we should only iterate non zero elements
  for (auto iter = mat.begin(); iter != mat.end(); ++iter) {
    int ii = iter.node()->idx[0];
    int jj = iter.node()->idx[1];
    uchar *data = iter.ptr;
    double v = 0.0;
    if (mat.type() == CV_32FC1) {
      float value = 0.0f;
      std::memcpy(&value, data, sizeof(value));
      v = value;
    } else if (mat.type() == CV_64FC1) {
      double value = 0.0f;
      std::memcpy(&value, data, sizeof(value));
      v = value;
    } else if (mat.type() == CV_32SC1) {
      int32_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      v = value;
    } else if (mat.type() == CV_64FC1) {
      int64_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      v = value;
    } else if (mat.type() == CV_8UC1) {
      uint8_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      v = value;
    } else {
      assert(false && "element type is not supported here!");
    }
    if (v != 0.0)
      triplets.emplace_back(jj, ii, v); // col - row - val
  }

  // make triplets ordered in column indices to satisfy matlab interface
  std::sort(triplets.begin(), triplets.end());

  // fill in matlab data
  auto sr = mxGetPr(ma);
  // auto si = mxGetPi(ma);
  auto irs = mxGetIr(ma);
  auto jcs = mxGetJc(ma);
  jcs = (mwIndex *)mxRealloc(jcs, (mat.size(1) + 1) * sizeof(mwIndex));
  std::fill(jcs, jcs + (mat.size(1) + 1), 0);
  mxSetJc(ma, jcs);

  for (int i = 0; i < triplets.size(); i++) {
    sr[i] = std::get<2>(triplets[i]);
    int ii = std::get<1>(triplets[i]) + 1;
    int jj = std::get<0>(triplets[i]) + 1;
    irs[i] = ii - 1;
    jcs[jj]++;
  }

  for (int j = 1; j < (mat.size(1) + 1); j++) {
    jcs[j] += jcs[j - 1];
  }

  _mxa = ma;
  _destroyWhenOutofScope = dos;
}

MXA::MXA(cv::InputArray m, bool dos /*= false*/) {
  cv::Mat im = m.getMat();
  // collect all dimensions of im
  int channelNum = im.channels();
  mwSize *dimSizes = new mwSize[im.dims + 1];
  for (int i = 0; i < im.dims; i++)
    dimSizes[i] = im.size[i];
  dimSizes[im.dims] = channelNum;

  // create mxArray
  mxArray *ma = mxCreateNumericArray(im.dims + 1, dimSizes,
                                     CVDepthToMxClassID(im.depth()), mxREAL);
  delete[] dimSizes;

  if (!ma)
    return;

  uint8_t *mad = (uint8_t *)mxGetData(ma);
  const size_t szForEachElem = im.elemSize1();
  mwIndex *mxIndices = new mwIndex[im.dims + 1];
  int *cvIndices = new int[im.dims];

  cv::MatConstIterator iter(&im);
  int imTotal = im.total();
  for (int i = 0; i < imTotal; i++, ++iter) {
    // get indices in cv::Mat
    iter.pos(cvIndices);
    // copy indices to mxIndices
    std::copy(cvIndices, cvIndices + im.dims, mxIndices);
    for (mwIndex k = 0; k < channelNum; k++) {
      const uint8_t *fromDataHead = (*iter) + k * szForEachElem;
      mxIndices[im.dims] = k; // set the last indices
      uint8_t *toDataHead =
          mad +
          mxCalcSingleSubscript(ma, im.dims + 1, mxIndices) * szForEachElem;
      std::memcpy(toDataHead, fromDataHead, szForEachElem);
    }
  }

  delete[] mxIndices;
  delete[] cvIndices;

  _mxa = ma;
  _destroyWhenOutofScope = dos;
}

MXA &MXA::operator=(MXA &&a) {
  std::swap(_mxa, a._mxa);
  std::swap(_destroyWhenOutofScope, a._destroyWhenOutofScope);
  return *this;
}
MXA::~MXA() {
  if (_destroyWhenOutofScope) {
    mxDestroyArray(static_cast<mxArray *>(_mxa));
  }
  _mxa = nullptr;
}

#define IMP_MXARRAY_MEMBERFUNCTION_IS(what)                                    \
  bool MXA::is##what() const {                                                 \
    return mxIs##what(static_cast<mxArray *>(_mxa));                           \
  }

IMP_MXARRAY_MEMBERFUNCTION_IS(Numeric)
IMP_MXARRAY_MEMBERFUNCTION_IS(Cell)
IMP_MXARRAY_MEMBERFUNCTION_IS(Logical)
IMP_MXARRAY_MEMBERFUNCTION_IS(Char)
IMP_MXARRAY_MEMBERFUNCTION_IS(Struct)
IMP_MXARRAY_MEMBERFUNCTION_IS(Opaque)
IMP_MXARRAY_MEMBERFUNCTION_IS(FunctionHandle)
IMP_MXARRAY_MEMBERFUNCTION_IS(Object)

IMP_MXARRAY_MEMBERFUNCTION_IS(Complex)
IMP_MXARRAY_MEMBERFUNCTION_IS(Sparse)
IMP_MXARRAY_MEMBERFUNCTION_IS(Double)
IMP_MXARRAY_MEMBERFUNCTION_IS(Single)
IMP_MXARRAY_MEMBERFUNCTION_IS(Int8)
IMP_MXARRAY_MEMBERFUNCTION_IS(Uint8)
IMP_MXARRAY_MEMBERFUNCTION_IS(Int16)
IMP_MXARRAY_MEMBERFUNCTION_IS(Uint16)
IMP_MXARRAY_MEMBERFUNCTION_IS(Int32)
IMP_MXARRAY_MEMBERFUNCTION_IS(Uint32)
IMP_MXARRAY_MEMBERFUNCTION_IS(Int64)
IMP_MXARRAY_MEMBERFUNCTION_IS(Uint64)

std::string MXA::toString() const {
  size_t len = nelements() + 1;
  char *buffer = new char[len];
  memset(buffer, '\0', len);
  mxGetString(static_cast<mxArray *>(_mxa), buffer, len);
  std::string str = buffer;
  delete[] buffer;
  return str;
}

bool MXA::toCVOutputArray(cv::OutputArray mat,
                          bool lastDimIsChannel /*= true*/) const {
  if (!mat.needed())
    return true;
  if (null())
    return false;
  const mxArray *ma = static_cast<mxArray *>(_mxa);
  int d = mxGetNumberOfDimensions(ma);
  assert((d >= 2) && "dimension num of the variable must be over 2");
  const mwSize *dimSizes = mxGetDimensions(ma);

  // set channels, dims and dimSizes
  int channels = 0;
  int cvDims = 0;
  int *cvDimSizes = nullptr;

  if (lastDimIsChannel && d > 2) {
    channels = dimSizes[d - 1];
    cvDims = d - 1;
    cvDimSizes = new int[d - 1];
    std::copy(dimSizes, dimSizes + d - 1, cvDimSizes);
  } else {
    channels = 1;
    cvDims = d;
    cvDimSizes = new int[d];
    std::copy(dimSizes, dimSizes + d, cvDimSizes);
  }

  const size_t szForEachElem = mxGetElementSize(ma);
  int depth = MxClassIDToCVDepth(mxGetClassID(ma));
  if (depth == -1) {
    delete[] cvDimSizes;
    return false;
  }

  const uint8_t *mad = (const uint8_t *)mxGetData(ma);
  // create Mat
  mat.create(cvDims, cvDimSizes, CV_MAKETYPE(depth, channels));
  cv::Mat im = mat.getMat();

  mwIndex *mxIndices = new mwIndex[im.dims + 1];
  int *cvIndices = new int[im.dims];

  cv::MatConstIterator iter(&im);
  int imTotal = im.total();
  for (int i = 0; i < imTotal; i++, ++iter) {
    // get indices in cv::Mat
    iter.pos(cvIndices);
    // copy indices to mxIndices
    std::copy(cvIndices, cvIndices + im.dims, mxIndices);
    for (mwIndex k = 0; k < channels; k++) {
      uint8_t *toDataHead = (uint8_t *)(*iter) + k * szForEachElem;
      mxIndices[im.dims] = k; // set the last indices
      const uint8_t *fromDataHead =
          mad +
          mxCalcSingleSubscript(ma, im.dims + 1, mxIndices) * szForEachElem;
      std::memcpy(toDataHead, fromDataHead, szForEachElem);
    }
  }

  delete[] cvDimSizes;
  delete[] mxIndices;
  delete[] cvIndices;
  return true;
}

MXA MXA::clone(bool dos) const {
  if (_mxa)
    return MXA(mxDuplicateArray(static_cast<mxArray *>(_mxa)), dos);
  return MXA();
}

void *MXA::data() const { return mxGetData(static_cast<mxArray *>(_mxa)); }

void MXA::setData(void *d) { mxSetData(static_cast<mxArray *>(_mxa), d); }

size_t MXA::m() const { return mxGetM(static_cast<mxArray *>(_mxa)); }

size_t MXA::n() const { return mxGetN(static_cast<mxArray *>(_mxa)); }

void MXA::setM(size_t m) { mxSetM(static_cast<mxArray *>(_mxa), m); }

void MXA::setN(size_t n) { mxSetN(static_cast<mxArray *>(_mxa), n); }

bool MXA::empty() const { return mxIsEmpty(static_cast<mxArray *>(_mxa)); }

bool MXA::isFromGlobalWorkspace() const {
  return mxIsFromGlobalWS(static_cast<mxArray *>(_mxa));
}

void MXA::setIsFromGlobalWorkspace(bool b) {
  mxSetFromGlobalWS(static_cast<mxArray *>(_mxa), b);
}

double MXA::scalar() const { return mxGetScalar(static_cast<mxArray *>(_mxa)); }

size_t MXA::nelements() const {
  return mxGetNumberOfElements(static_cast<mxArray *>(_mxa));
}

size_t MXA::nzmax() const { return mxGetNzmax(static_cast<mxArray *>(_mxa)); }

MXA MXA::cell(size_t i) const {
  return mxGetCell(static_cast<mxArray *>(_mxa), i);
}

void MXA::setCell(size_t i, const MXA &a) {
  mxSetCell(static_cast<mxArray *>(_mxa), i, static_cast<mxArray *>(a._mxa));
}

int MXA::nfields() const {
  return mxGetNumberOfFields(static_cast<mxArray *>(_mxa));
}

const char *MXA::fieldName(int n) const {
  return mxGetFieldNameByNumber(static_cast<mxArray *>(_mxa), n);
}

int MXA::fieldNumber(const std::string &name) const {
  return mxGetFieldNumber(static_cast<mxArray *>(_mxa), name.c_str());
}

MXA MXA::field(const std::string &name, int i /*= 0*/) const {
  return mxGetField(static_cast<mxArray *>(_mxa), i, name.c_str());
}

void MXA::setField(const std::string &name, int i, const MXA &a) {
  mxSetField(static_cast<mxArray *>(_mxa), i, name.c_str(),
             static_cast<mxArray *>(a._mxa));
}

MXA MXA::property(const std::string &name, size_t i) const {
  return mxGetProperty(static_cast<mxArray *>(_mxa), i, name.c_str());
}

void MXA::setProperty(const std::string &name, int i, const MXA &a) {
  mxSetProperty(static_cast<mxArray *>(_mxa), i, name.c_str(),
                static_cast<mxArray *>(a._mxa));
}

const char *MXA::className() const {
  return mxGetClassName(static_cast<mxArray *>(_mxa));
}

size_t MXA::ndims() const {
  return mxGetNumberOfDimensions(static_cast<mxArray *>(_mxa));
}

std::vector<size_t> MXA::dims() const {
  std::vector<size_t> ds(ndims());
  std::copy_n(mxGetDimensions(static_cast<mxArray *>(_mxa)), ds.size(),
              ds.begin());
  return ds;
}

size_t MXA::length() const {
  auto ds = dims();
  if (ds.empty())
    return 0;
  return *std::max_element(ds.begin(), ds.end());
}

size_t MXA::calcSingleSubscript(size_t dim, size_t *subs) const {
  return mxCalcSingleSubscript(static_cast<mxArray *>(_mxa), dim, subs);
}

MXA MXA::createString(const std::string &str, bool dos) {
  return MXA(mxCreateString(str.c_str()), dos);
}

MXA MXA::createCellMatrix(int m, int n, bool dos) {
  return MXA(mxCreateCellMatrix(m, n), dos);
}

MXA MXA::createStructMatrix(int m, int n,
                            const std::vector<std::string> &fieldNames,
                            bool dos) {
  char const **fieldNamesArray = new char const *[fieldNames.size()];
  for (int i = 0; i < fieldNames.size(); i++) {
    fieldNamesArray[i] = fieldNames[i].c_str();
  }
  MXA mxa(mxCreateStructMatrix(m, n, fieldNames.size(), fieldNamesArray), dos);
  delete[] fieldNamesArray;
  return mxa;
}

MAT::MAT() : _fp(nullptr) {}
MAT::MAT(const std::string &fname, const std::string &mode) : _fname(fname) {
  _fp = matOpen(fname.c_str(), mode.c_str());
}
MAT::MAT(const std::string &fname, OpeningMode mode)
    : _fname(fname), _fp(nullptr) {
  std::string m;
  switch (mode) {
  case Read:
    m = "r";
    break;
  case Update:
    m = "u";
    break;
  case Write:
    m = "w";
    break;
  case Write_4:
    m = "w4";
    break;
  case Write_CompressedData:
    m = "wz";
    break;
  case Write_7_3:
    m = "w7.3";
    break;
  default:
    return;
  }
  _fp = matOpen(fname.c_str(), m.c_str());
}

MAT::MAT(MAT &&a) {
  _fname = a._fname;
  a._fname.clear();
  _fp = a._fp;
  a._fp = nullptr;
}
MAT &MAT::operator=(MAT &&a) {
  std::swap(_fname, a._fname);
  std::swap(_fp, a._fp);
  return *this;
}

MAT::~MAT() {
  if (_fp) {
    matClose(static_cast<::MATFile *>(_fp));
    _fp = nullptr;
  }
}

std::vector<std::string> MAT::varNames() const {
  int num = 0;
  char **names = matGetDir(static_cast<::MATFile *>(_fp), &num);
  if (!names)
    return std::vector<std::string>();
  std::vector<std::string> vnames(num);
  for (int i = 0; i < num; i++) {
    vnames[i] = names[i];
  }
  mxFree(names);
  return vnames;
}

MXA MAT::var(const std::string &name) const {
  return matGetVariable(static_cast<::MATFile *>(_fp), name.c_str());
}

bool MAT::setVar(const std::string &name, const MXA &mxa,
                 bool asGlobal /*= false*/) {
  if (!asGlobal) {
    return matPutVariable(static_cast<::MATFile *>(_fp), name.c_str(),
                          static_cast<mxArray *>(mxa.mxa())) == 0;
  } else {
    return matPutVariableAsGlobal(static_cast<::MATFile *>(_fp), name.c_str(),
                                  static_cast<mxArray *>(mxa.mxa())) == 0;
  }
}

bool MAT::removeVar(const std::string &name) {
  return matDeleteVariable(static_cast<::MATFile *>(_fp), name.c_str()) == 0;
}

Matlab::Matlab(const std::string &defaultDir, bool singleUse, bool printMsg)
    : _eng(nullptr), _buffer(nullptr), _printMessage(printMsg) {
  static const int bufferSize = 1024;
  if (singleUse) {
    _eng = engOpenSingleUse(nullptr, nullptr, nullptr);
  } else {
    _eng = engOpen(nullptr);
  }
  if (_eng) {
    engSetVisible(static_cast<::Engine *>(_eng), false);
    _buffer = new char[bufferSize];
    std::memset(_buffer, 0, bufferSize);
    engOutputBuffer(static_cast<::Engine *>(_eng), _buffer, bufferSize);
    if (_printMessage) {
      std::cout << "Matlab Engine Launched" << std::endl;
    }
    if (!defaultDir.empty()) {
      engEvalString(static_cast<::Engine *>(_eng),
                    ("cd " + defaultDir + "; startup; pwd").c_str());
    } else {
      (*this) << (std::string("cd ") + PANORAMIX_MATLAB_CODE_DIR_STR +
                  "; startup; pwd");
    }
    if (_printMessage) {
      std::cout << _buffer << std::endl;
    }
  }
}

Matlab::~Matlab() {
  if (_eng) {
    engClose(static_cast<::Engine *>(_eng));
    if (_printMessage) {
      std::cout << "Matlab Engine Closed" << std::endl;
    }
  }
  delete[] _buffer;
  _eng = nullptr;
  _buffer = nullptr;
}

Matlab::Matlab(Matlab &&e) {
  _eng = e._eng;
  e._eng = nullptr;
  _buffer = e._buffer;
  e._buffer = nullptr;
}

Matlab &Matlab::operator=(Matlab &&e) {
  std::swap(e._eng, _eng);
  std::swap(e._buffer, _buffer);
  return *this;
}

bool Matlab::started() const { return _eng != nullptr; }

bool Matlab::run(const std::string &cmd) const {
  bool ret = engEvalString(static_cast<::Engine *>(_eng), cmd.c_str()) == 0;
  if (_printMessage && strlen(_buffer) > 0) {
    std::cout << "[Message when executing '" << cmd << "']:\n" << _buffer
              << std::endl;
  }
  return ret;
}

std::string Matlab::lastMessage() const { return _buffer; }

bool Matlab::errorLastRun() const {
  return std::string(_buffer).substr(0, 5) == "Error";
}

MXA Matlab::var(const std::string &name) const {
  return MXA(engGetVariable(static_cast<::Engine *>(_eng), name.c_str()), true);
}

bool Matlab::setVar(const std::string &name, const MXA &mxa) {
  return engPutVariable(static_cast<::Engine *>(_eng), name.c_str(),
                        static_cast<mxArray *>(mxa.mxa())) == 0;
}

const Matlab &Matlab::operator<<(const std::string &cmd) const {
  run(cmd);
  return *this;
}

bool Matlab::cdAndAddAllSubfolders(const std::string &dir) {
  return run("cd " + dir) && run("addpath(genpath('.'));");
}
}
}
