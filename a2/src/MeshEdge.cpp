#include "MeshEdge.hpp"
#include "MeshFace.hpp"
#include "MeshVertex.hpp"
#include "DGP/Vector4.hpp"

MeshEdge *
MeshEdge::nextAroundEndpoint(int i)
{
  debugAssertM(i == 0 || i == 1, "MeshEdge: Invalid endpoint index");

  if (numFaces() > 2)  // non-manifold
    return NULL;

  // Find which incident face has this endpoint as the origin of the edge when stepping round the face. The required edge
  // is then the predecessor of this edge around the face.
  for (FaceIterator fi = facesBegin(); fi != facesEnd(); ++fi)
  {
    Face * face = *fi;
    MeshEdge * prev = face->getPredecessor(this);
    if (prev->hasEndpoint(endpoints[i]))  // found it!
      return prev;
  }

  return NULL;
}

void
MeshEdge::updateQuadricCollapseError()
{
  // TODO

  // Update both quadric_collapse_error and quadric_collapse_position, using the existing endpoint quadrics and the method of
  // Garland/Heckbert.
  //
  // NOTE: Remember to check if the quadric Q' is invertible. If not, you will have to use a fallback option such as the
  // midpoint of the edge (or in the worst case, set the error to a negative value to indicate this edge should not be
  // collapsed).
  DMat4 quadric = endpoints[0]->getQuadric() + endpoints[1]->getQuadric();
  // DMat4 quadric2;
  // quadric2 = quadric;
  double identity[4]={0,0,0,1};
  // quadric2.setRow(3,identity);
  MatrixMN<4,1,double> v_dash;
  if(quadric.determinant() <=0.00001){
    quadric_collapse_position = (endpoints[0]->getPosition()+endpoints[1]->getPosition())/2;
    v_dash(0,0) = quadric_collapse_position.x();
    v_dash(1,0) = quadric_collapse_position.y();
    v_dash(2,0) = quadric_collapse_position.z();
    v_dash(3,0) = 1;
  }
  else{
    MatrixMN<4,1,double> ident_mat;
    ident_mat.setColumn(0,identity);
    v_dash = quadric.inverse()*ident_mat;
    if (v_dash(3,0) <= 0.000001)
    {
      quadric_collapse_position = (endpoints[0]->getPosition()+endpoints[1]->getPosition())/2;
      v_dash(0,0) = quadric_collapse_position.x();
      v_dash(1,0) = quadric_collapse_position.y();
      v_dash(2,0) = quadric_collapse_position.z();
      v_dash(3,0) = 1;
    }
    else{
      v_dash /= v_dash(3,0);
      quadric_collapse_position = Vector3(v_dash(0,0),v_dash(1,0),v_dash(2,0));
    }
  }
  quadric_collapse_error = (v_dash.transpose()*quadric*v_dash)(0,0);
  return;
}
