/*
 *  Face.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef Face_h
#define Face_h
/*
struct Vertex
{
   double x;
   double y;
   double z;
};

class Face
{
   public:
      Face();
      ~Face();

      int GetVertex(int i)
      {
         if (i < 0 || i >= mNumVertices) throw (__LINE__);
         return mVertexList[i];
      }

      void SetVertex(int i, int v)
      {
         if (i < 0 || i >= mNumVertices) throw (__LINE__);
         mVertexList[i] = v;
      }

      int GetNumVertices()
      {
         return mNumVertices;
      }

      void SetNumVertices(int numVertices);

      Face& operator=(Face &in);

      void SetNormal(double x, double y, double z)
      {
         mNormal[0] = x;
         mNormal[1] = y;
         mNormal[2] = z;
      }

      void GetNormal(double normal[3])
      {
         normal[0] = mNormal[0];
         normal[1] = mNormal[1];
         normal[2] = mNormal[2];
      }

      void GetNormal(float normal[3])
      {
         normal[0] = mNormal[0];
         normal[1] = mNormal[1];
         normal[2] = mNormal[2];
      }

      void ReverseVertexOrder();
      void OffsetVertices(int offset);

   protected:
      int mNumVertices;
      int *mVertexList;
      double mNormal[3];
};
*/
#endif

