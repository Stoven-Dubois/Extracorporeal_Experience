using Demonixis.Kinect.Server;
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Demonixis.Server.Kinect
{
    class KinectSet
    {
        //Accessible data
        private KinectData[] _kinectDataSet; //Kinects data
        private int _numberOfKinects; //Number of kinects
        private int _numberOfPoints; //Number of points in the pointcloud
        private float _xMin, _xMax, _yMin, _yMax, _zMin, _zMax; //Ranges in the original system defining the available zone
        private float _cubeSize; //Cubes size for the grid in meters
        private JsonPoint[] _imageAfterMerge; //Pointcloud merged from the different Kinects' pointclouds

        //Non-Accessible data
        private int _numberOfZonesX, _numberOfZonesY, _numberOfZonesZ, _numberOfZonesTotal; //Sizes of the grid
        private int _numberOfPointsPerKinect; //Number of points contained in one kinect
        private float[] _tempPositionsMerge; //Current positions in respective methods
        private float[] _tempColorsMerge; //Current colors in respective methods
        private int _xMerge, _yMerge, _zMerge, _indiceMerge; //Coordinates and indice of current pixel in merge grid
        private int _maxIndiceMerge, _numberOfPointsMerge; //Data of the merge grid
        private int _cpt; //Counter
        private int _currentIndiceNeighbour; //Current index of the neighbour
        private int[] _correspondingIndex; //Corresponding index with reduction of size
        private float _factorOfImportance; //Merge parameters
        private float[] _numberOfPointsAddedMerge; //Table containing number of pixels per cube in the merge grid

        public KinectSet(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax, float cubeSize)
        {
            _numberOfKinects = 0;
            _xMin = xMin; _xMax = xMax;
            _yMin = yMin; _yMax = yMax;
            _zMin = zMin; _zMax = zMax;
            _cubeSize = cubeSize;
            _imageAfterMerge = null;

            _numberOfPointsPerKinect = 0;
            _numberOfZonesX = (int)(Math.Ceiling(((_xMax - _xMin) / _cubeSize))) + 1;
            _numberOfZonesY = (int)(Math.Ceiling(((_yMax - _yMin) / _cubeSize))) + 1;
            _numberOfZonesZ = (int)(Math.Ceiling(((_zMax - _zMin) / _cubeSize))) + 1;
            _numberOfZonesTotal = (_numberOfZonesX+10) * (_numberOfZonesY+10) * (_numberOfZonesZ+10);
            _tempPositionsMerge = new float[3*_numberOfZonesTotal];
            _tempColorsMerge = new float[3*_numberOfZonesTotal];
            _correspondingIndex = new int[_numberOfZonesTotal];
            _xMerge = 0; _yMerge = 0; _zMerge = 0;
            _maxIndiceMerge = 0;
            _numberOfPointsMerge = 0;
            _factorOfImportance = 0;
            _numberOfPointsAddedMerge = new float[_numberOfZonesTotal];
        }
        public int getNumberOfPoints()
        {
            return _numberOfPoints;
        }

        public int getNumberOfKinects()
        {
            return _numberOfKinects;
        }

        public KinectData[] getKinectDataSet()
        {
            return _kinectDataSet;
        }

        public KinectData getKinectData(int indice)
        {
            return _kinectDataSet[indice];
        }

        public int returnNumberOfKinects()
        {
            return _numberOfKinects;
        }

        public int returnNumberOfPoints()
        {
            return _numberOfPoints;
        }

        public float getXMin()
        {
            return _xMin;
        }

        public float getXMax()
        {
            return _xMax;
        }

        public float getYMin()
        {
            return _yMin;
        }

        public float getYMax()
        {
            return _yMax;
        }

        public float getZMin()
        {
            return _zMin;
        }

        public float getZMax()
        {
            return _zMax;
        }

        public float getCubeSize()
        {
            return _cubeSize;
        }

        public JsonPoint[] getImageAfterMerge()
        {
            return _imageAfterMerge;
        }

        public void addKinect(KinectData kinectData)
        //Adds a Kinect to the set
        {
             KinectData[] newKinectSet = new KinectData[_numberOfKinects + 1];
             for (int i = 0; i < _numberOfKinects; i++)
             {
                 newKinectSet[i] = _kinectDataSet[i];
             }
             newKinectSet[_numberOfKinects] = kinectData;
             _kinectDataSet = newKinectSet;
             _numberOfKinects++;
             Console.WriteLine("Adding : Kinect {0}", _numberOfKinects);
         }

         public void mergeKinects()
         //Merge data from Kinects using a parametered voxel grid filter
         {
             _maxIndiceMerge = 0;
             for (int i = 0; i < _numberOfKinects; i++)
             {
                 _numberOfPointsPerKinect = _kinectDataSet[i].getNumberOfPoints();
                 for (int j = 0; j < _numberOfPointsPerKinect; j++)
                 {
                     _xMerge = (int)Math.Floor((_kinectDataSet[i].getImageAfterFiltering()[j].x - _xMin) / _cubeSize);
                     _yMerge = (int)Math.Floor((_kinectDataSet[i].getImageAfterFiltering()[j].y - _yMin) / _cubeSize);
                     _zMerge = (int)Math.Floor((_kinectDataSet[i].getImageAfterFiltering()[j].z - _zMin) / _cubeSize);
                     if (_xMerge == _numberOfZonesX)
                     {
                         _xMerge--;
                     }
                     if (_yMerge == _numberOfZonesY)
                     {
                         _yMerge--;
                     }

                     if (_zMerge == _numberOfZonesZ)
                     {
                         _zMerge--;
                     }


                     _indiceMerge = (_xMerge) + (_numberOfZonesX * (_yMerge)) + ((_numberOfZonesX * _numberOfZonesY) * (_zMerge));
                    if (_indiceMerge > _maxIndiceMerge)
                     {
                         _maxIndiceMerge = _indiceMerge;
                     }
                    if (true)
                    {
                        _factorOfImportance = _kinectDataSet[i].getStandardDeviationAfterFiltering()[j];
                        _tempPositionsMerge[3 * _indiceMerge] = _tempPositionsMerge[3 * _indiceMerge] + _factorOfImportance * _kinectDataSet[i].getImageAfterFiltering()[j].x;
                        _tempPositionsMerge[3 * _indiceMerge + 1] = _tempPositionsMerge[3 * _indiceMerge + 1] + _factorOfImportance * _kinectDataSet[i].getImageAfterFiltering()[j].y;
                        _tempPositionsMerge[3 * _indiceMerge + 2] = _tempPositionsMerge[3 * _indiceMerge + 2] + _factorOfImportance * _kinectDataSet[i].getImageAfterFiltering()[j].z;
                        _tempColorsMerge[3 * _indiceMerge] = ((_tempColorsMerge[3 * _indiceMerge] + _factorOfImportance * _kinectDataSet[i].getImageAfterFiltering()[j].r));
                        _tempColorsMerge[3 * _indiceMerge + 1] = ((_tempColorsMerge[3 * _indiceMerge + 1] + _factorOfImportance * _kinectDataSet[i].getImageAfterFiltering()[j].g));
                        _tempColorsMerge[3 * _indiceMerge + 2] = ((_tempColorsMerge[3 * _indiceMerge + 2] + _factorOfImportance * _kinectDataSet[i].getImageAfterFiltering()[j].b));
                        _numberOfPointsAddedMerge[_indiceMerge] = _numberOfPointsAddedMerge[_indiceMerge] + _factorOfImportance;
                    }
                 }
             }
            _numberOfPointsMerge = 0;
             for (int i = 0; i < _numberOfZonesTotal; i++)
             {
                 if (_numberOfPointsAddedMerge[i] > 0)
                 {
                     _tempPositionsMerge[3*i] = _tempPositionsMerge[3*i] / _numberOfPointsAddedMerge[i];
                     _tempPositionsMerge[3*i+1] = _tempPositionsMerge[3*i+1] / _numberOfPointsAddedMerge[i];
                     _tempPositionsMerge[3*i+2] = _tempPositionsMerge[3*i+2] / _numberOfPointsAddedMerge[i];
                     _tempColorsMerge[3*i] = (_tempColorsMerge[3*i]) / _numberOfPointsAddedMerge[i];
                     _tempColorsMerge[3*i+1] = (_tempColorsMerge[3*i+1]) / _numberOfPointsAddedMerge[i];
                     _tempColorsMerge[3*i+2] = (_tempColorsMerge[3*i+2]) / _numberOfPointsAddedMerge[i];
                     _numberOfPointsMerge++;
                 }
             }
            _numberOfPoints = _numberOfPointsMerge;


            _imageAfterMerge = new JsonPoint[_numberOfPoints];

            _cpt = 0;
             for (int i = 0; i < _numberOfZonesTotal; i++)
             {
                 if (_numberOfPointsAddedMerge[i] > 0)
                 {
                    _imageAfterMerge[_cpt].x = _tempPositionsMerge[3 * i];
                    _imageAfterMerge[_cpt].y = _tempPositionsMerge[3 * i + 1];
                    _imageAfterMerge[_cpt].z = _tempPositionsMerge[3 * i + 2];
                    _imageAfterMerge[_cpt].r = (byte)_tempColorsMerge[3 * i];
                    _imageAfterMerge[_cpt].g = (byte)_tempColorsMerge[3 * i + 1];
                    _imageAfterMerge[_cpt].b = (byte)_tempColorsMerge[3 * i + 2];
                    _imageAfterMerge[_cpt].neighbours = new int[27];
                    _correspondingIndex[i] = _cpt;
                    for (int j = -1; j <= 1; j++)
                    {
                        for(int k=-1; k <= 1; k++)
                        {
                            for(int l = -1; l <= 1; l++)
                            {
                                if(j!=0 || k!=0 || l != 0)
                                {
                                    _currentIndiceNeighbour = i+(j)+(_numberOfZonesX * (k))+((_numberOfZonesX * _numberOfZonesY)*l);
                                    if(_currentIndiceNeighbour>=0 && _currentIndiceNeighbour<_maxIndiceMerge && _numberOfPointsAddedMerge[_currentIndiceNeighbour] > 0)
                                    {
                                        _imageAfterMerge[_cpt].neighbours[9 * (j + 1) + 3 * (k + 1) + (l + 1)] = _currentIndiceNeighbour;
                                    }else
                                    {
                                        _imageAfterMerge[_cpt].neighbours[9 * (j+1) + 3 * (k+1) + (l+1)] = 0;
                                    }
                                }else
                                {
                                    _imageAfterMerge[_cpt].neighbours[9 * (j + 1) + 3 * (k + 1) + (l + 1)] = 0;
                                }
                            }
                        }
                    }
                    _tempPositionsMerge[3 * i] = 0;
                    _tempPositionsMerge[3 * i + 1] = 0;
                    _tempPositionsMerge[3 * i + 2] = 0;
                    _tempColorsMerge[3 * i] = 0;
                    _tempColorsMerge[3 * i + 1] = 0;
                    _tempColorsMerge[3 * i + 2] = 0;
                    _tempColorsMerge[i] = 0;
                    _numberOfPointsAddedMerge[i] = 0;
                    _cpt++;
                 }
             }

             for(int i = 0; i < _numberOfPoints-1; i++)
             {
                for(int j = 0; j < 27; j++)
                {
                    if (_imageAfterMerge[i].neighbours[j] > 0)
                    {
                        _imageAfterMerge[i].neighbours[j] = _correspondingIndex[_imageAfterMerge[i].neighbours[j]];
                    }
                }
             }
        }

         public JsonDepth ToJsonFormat()
         {
            var dataToSend = new JsonDepth()
             {
                 Id = _numberOfKinects,
                 NbPoints = _numberOfPoints,
                 PointCloud = _imageAfterMerge
             };
             return dataToSend;
         }
    }
}