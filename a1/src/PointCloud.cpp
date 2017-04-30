#include "PointCloud.hpp"
#include "DGP/Graphics/Shader.hpp"
#include <fstream>
#include <sstream>

PointCloud::PointCloud(std::vector<Point> const & points_)
: points(points_)
{
  recomputeAABB();
}

PointCloud::PointCloud(std::vector<Vector3> const & positions, std::vector<Vector3> const & normals)
{
  alwaysAssertM(positions.size() == normals.size(), "PointCloud: Number of positions != number of normals");

  for (size_t i = 0; i < positions.size(); ++i)
    points.push_back(Point(positions[i], normals[i]));

  recomputeAABB();
}

bool
PointCloud::load(std::string const & path)
{
  // Simple format: Each line is either
  //   x y z
  //    OR
  //   x y z nx ny nz
  //
  // where (nx, ny, nz) is the normal

  std::ifstream in(path.c_str());
  if (!in)
  {
    DGP_ERROR << "Could not open file for reading: " << path;
    return false;
  }

  std::string line;
  while (getline(in, line))
  {
    // Skip empty lines
    line = trimWhitespace(line);
    if (line.empty())
      continue;

    std::istringstream line_in(line);
    Vector3 p;
    if (!(line_in >> p[0] >> p[1] >> p[2]))
    {
      DGP_ERROR << "Could not read point " << points.size() << " from line: " << line;
      return false;
    }

    // Normal is optional
    Vector3 n;
    if (!(line_in >> n[0] >> n[1] >> n[2]))  // doesn't properly handle malformed lines, but we'll ignore this for now
      n = Vector3::zero();

    points.push_back(Point(p, n));
  }

  recomputeAABB();

  return true;
}

bool
PointCloud::save(std::string const & path) const
{
  std::ofstream out(path.c_str(), std::ios::binary);
  if (!out)
  {
    DGP_ERROR << "Could not open file for writing: " << path;
    return false;
  }

  for (size_t i = 0; i < points.size(); ++i)
  {
    Vector3 const & p = points[i].getPosition();
    Vector3 const & n = points[i].getNormal();
    out << p[0] << ' ' << p[1] << ' ' << p[2] << ' ' << n[0] << ' ' << n[1] << ' ' << n[2] << '\n';
  }

  return true;
}

Graphics::Shader *
createPointShader(Graphics::RenderSystem & rs)
{
  static std::string const VERTEX_SHADER =
"void main()\n"
"{\n"
"  gl_Position = ftransform();\n"
"  gl_FrontColor = gl_Color;\n"
"  gl_BackColor = gl_Color;\n"
"}\n";

  static std::string const FRAGMENT_SHADER =
"void main()\n"
"{\n"
"  gl_FragColor = gl_Color;\n"
"}\n";

  Graphics::Shader * shader = rs.createShader("Point Graphics::Shader");
  if (!shader)
    throw Error("Could not create point shader");

  // Will throw errors on failure
  shader->attachModuleFromString(Graphics::Shader::ModuleType::VERTEX, VERTEX_SHADER.c_str());
  shader->attachModuleFromString(Graphics::Shader::ModuleType::FRAGMENT, FRAGMENT_SHADER.c_str());

  return shader;
}

void
PointCloud::draw(Graphics::RenderSystem & rs, Real normal_len) const
{
  // Make this static to ensure just one shader is created. Assumes rendersystem is constant, not the best design pattern.
  static Graphics::Shader * shader = createPointShader(rs);

  rs.pushShader();
  rs.pushColorFlags();
  rs.pushShapeFlags();

    rs.setShader(shader);
    rs.setPointSize(2.0f);

    Vector3 lo = bbox.getLow();
    Vector3 ext = bbox.getExtent();

    rs.beginPrimitive(Graphics::RenderSystem::Primitive::POINTS);
      for (size_t i = 0; i < points.size(); ++i)
      {
        if (!points[i].display)
        {
          continue;
        }
        Vector3 rel_pos = (points[i].getPosition() - lo) / ext;
        rs.setColor(ColorRGB(1,1,1));//points[i].color.x(),points[i].color.y(),points[i].color.z()));
        rs.sendVertex(points[i].getPosition());
      }
    rs.endPrimitive();

    if (normal_len > 0)
    {
      rs.setColor(ColorRGB(0.5, 0.5, 1.0));  // blue

      rs.beginPrimitive(Graphics::RenderSystem::Primitive::LINES);
        for (size_t i = 0; i < points.size(); ++i)
        {
          Vector3 const & p = points[i].getPosition();
          Vector3 const & n = points[i].getNormal();

          rs.sendVertex(p);
          rs.sendVertex(p + normal_len * n);
        }
      rs.endPrimitive();
    }

  rs.popShapeFlags();
  rs.popColorFlags();
  rs.popShader();
}

void
PointCloud::recomputeAABB()
{
  bbox.setNull();

  for (size_t i = 0; i < points.size(); ++i)
    bbox.merge(points[i].getPosition());
}

long
PointCloud::ransac(long num_iters, Real slab_thickness, long min_points, Slab & slab, std::vector<Point *> & slab_points) const
{
  // TODO

  //   - Construct a kd-tree on the enabled points (remember to build the kd-tree with pointers to existing points -- you
  //     shouldn't be copying the points themselves, either explicitly or implicitly).
  //   - Generate num_iters random triplets of enabled points and fit a plane to them.
  //   - Using the kd-tree, see how many other enabled points are contained in the slab supported by this plane with thickness
  //     slab_thickness (extends to distance 0.5 * slab_thickness on each side of the plane).
  //   - If this number is >= min_points and > the previous maximum, the plane is the current best fit. Set the 'slab' argument
  //     to be the slab for this plane, and update slab_points to be the set of (enabled) matching points for this plane.
  //   - At the end, for visualization purposes, update the corners of the best slab using its set of matching points, and
  //     return the number of (enabled) matching points.

  return 0;
}

long
PointCloud::ransacMultiple(long num_planes, long num_iters, Real slab_thickness, long min_points, std::vector<Slab> & slabs)
const
{
  for (size_t i = 0; i < points.size(); ++i)
    points[i].setEnabled(true);

  slabs.clear();
  for (long i = 0; i < num_planes; ++i)
  {
    Slab slab;
    std::vector<Point *> slab_points;

    long num_matching_pts = ransac(num_iters, slab_thickness, min_points, slab, slab_points);
    if (num_matching_pts <= 0)
      break;

    slabs.push_back(slab);

    // Don't consider these points in subsequent slabs
    for (size_t j = 0; j < slab_points.size(); ++j)
      slab_points[j]->setEnabled(false);
  }

  return (long)slabs.size();
}

void
PointCloud::adaptiveDownsample(std::vector<Slab> const & slabs)
{
  // TODO
}

class kmeansGroup{
public:
  std::vector<int> vertexIncluded;
  Vector3 center;
  kmeansGroup(){}
  kmeansGroup(Vector3 position){
    center = position;
  }
  Real distance(Vector3 const & position){
    return (center-position).length();
  }
  bool refreshCenter(std::vector<Point> const & points){
    Vector3 newCenter;
    if (vertexIncluded.size() <= 0)
    {
      return false;
    }
    for (int i = 0; i < vertexIncluded.size(); ++i)
    {
      newCenter += points[vertexIncluded[i]].getPosition();
    }
    newCenter = newCenter/vertexIncluded.size();
    if ((center-newCenter).length()> 0.1)
    {
      center = newCenter;
      return true;
    }
    return false;
  }
};

void PointCloud::kMeansVertexRemove(long k1){
  // printf("called\n");
  int a = cbrt(k1),b = cbrt(k1), c = k1/(a*b); 
  int k = a*b*c;
  kmeansGroup vertexCluster[k];
  int cluster_index = 0;
  Vector3 low = bbox.getLow();
  Vector3 high = bbox.getHigh();
  Vector3 diff = high-low;
  diff.x() = diff.x()/a;
  diff.y() = diff.y()/b;
  diff.z() = diff.z()/c;
  for (int i = 0; i < a; ++i)
  {
    for (int j = 0; j < b; ++j)
    {
      for (int k = 0; k < c; ++k)
      {
        // DGP_CONSOLE<<low.x()+diff.x()*(i+0.5)<<" "<<low.y()+diff.y()*(j+0.5)<<" "<<low.z()+diff.z()*(k + 0.5)<< std::endl;
        vertexCluster[cluster_index].center = Vector3(low.x()+diff.x()*(i+0.5),low.y()+diff.y()*(j+0.5),low.z()+diff.z()*(k + 0.5));
        cluster_index++;
      }
    }
  }
  bool change = true;
  // printf("1\n");
  while(change){
    // DGP_CONSOLE<<count_print++;
    for (int i = 0; i < k; ++i)
    {
      vertexCluster[i].vertexIncluded.clear();
    }
    int num_points = 0;
    for (int vi = 0; vi <points.size(); ++vi)
    {
      if (!points[vi].display)
      {
        continue;
      }
      num_points++;
      Real minDist = 10000000000000000;
      int index = -1;
      for (int i = 0; i < k; ++i)
      {
        // DGP_CONSOLE<<vertexCluster[i].distance(points[vi].getPosition());
        if(vertexCluster[i].distance(points[vi].getPosition())<minDist){
          minDist = vertexCluster[i].distance(points[vi].getPosition());
          index = i;
        }
      }
      if (index == -1)
      {
        printf("no cluster found\n");
        return;
      }
      else{
        vertexCluster[index].vertexIncluded.push_back(vi);
      }
    }
    if (num_points < 3)
    {
      return;
    }
    change = false;
    for (int i = 0; i < k; ++i)
    {
      if (vertexCluster[i].refreshCenter(points))
      {
        change = true;
      }
    }
  }
  // printf("2\n");
  int maxCount = 0,index = -1;
  for (int i = 0; i < k; ++i)
  {
    float color1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float color2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float color3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    for (int j = 0; j < vertexCluster[i].vertexIncluded.size(); ++j)
    {
      points[vertexCluster[i].vertexIncluded[j]].color = Vector3(color1,color2,color3);
    }
    if(vertexCluster[i].vertexIncluded.size()>maxCount){
      maxCount = vertexCluster[i].vertexIncluded.size();
      index = i;
    }
  }
  if (index == -1)
  {
    return;
  }
  // printf("%d\n",maxCount );
  for (int i = 0; i < maxCount; i+=2)
  {
    points[vertexCluster[index].vertexIncluded[i]].display = false;
  }
  // for (int i = 0; i < k; ++i)
  // {
  //   DGP_CONSOLE<<vertexCluster[i].vertexIncluded.size()<<", "<<vertexCluster[i].center;
  // }  
  return;
}