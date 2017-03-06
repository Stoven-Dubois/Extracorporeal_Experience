using Demonixis.Kinect.Server;
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static Demonixis.Server.Kinect.MatrixTransformation;

namespace Demonixis.Server.Kinect
{
    struct KinectImage
    {
        public JsonPoint[] _image { get; set; }
        public int _numberOfPoints { get; set; }
    }

    //2D-Image resolution
    public struct Resolution
    {
        public int _x { get; set; }
        public int _y { get; set; }

        public Resolution(int x, int y)
        {
            _x = x;
            _y = y;
        }
    }

    //Calibration
    public struct Calibration
    {
        public float _C_x { get; set; }
        public float _C_y { get; set; }
        public float _F_x { get; set; }
        public float _F_y { get; set; }

        public Calibration(float C_x, float C_y, float F_x, float F_y)
        {
            _C_x = C_x;
            _C_y = C_y;
            _F_x = F_x;
            _F_y = F_y;
        }
    }

    class KinectData
    {
        //Accessible data
        private int _id; //Id of the Kinect
        private int _numberOfPoints; //Number of points in the pointcloud
        private JsonPoint[] _imageBeforeFiltering, _imageAfterFiltering; //Pointcloud
        private float[] _standardDeviationsBeforeFiltering, _standardDeviationsAfterFiltering; //Standard deviations
        private Pose _position; //Position of the Kinect in original system
        private MatrixTransformation _matrixTransformation; //Transformation Matrix from original to Kinect system
        private Resolution _resolution; //Resolution of the point cloud (lowest resolution between the depth and color images)
        private Calibration _calibration; //Calibration data 
        private float _focalLength, _baseLength; //Specific calibration parameters of the Kinect
        private Byte[] _colorValues; //Data from the color image
        private DepthImagePixel[] _depthValues; //Data from the depth image
        private float _xMin, _xMax, _yMin, _yMax, _zMin, _zMax; //Ranges in the original system defining the available zone
        private float _cubeSize; //Cubes size for the grid in meters
        private float _minAngleVertical, _maxAngleVertical, _minAngleHorizontal, _maxAngleHorizontal;

        //Non-Accessible data
        private Vector3 _vectorMin, _vectorMax; //Vectors containing the limit ranges in the Kinect system
        private Vector3 _positionPixel; //Position of the current Pixel
        private JsonPoint[] _tempImageGetData; //Current pointclouds in respective methods
        private float[] _tempPositionsVoxelGrid; //Current positions in respective methods
        private int[] _tempColorsVoxelGrid; //Current pointclouds in respective methods
        private float[] _tempStandardDeviationGetData, _tempStandardDeviationVoxelGrid; //Current standard deviations in respective methods
        private int _currentNumberOfPoints; //Current number of points
        private float _currentX, _currentY, _currentZ; //Current coordinates of one point in the Kinect axis system
        private float _currentStandardDeviation; //Current likelihood of one point
        private float _currentAngleVertical, _currentAngleHorizontal; //Current angles of one point over the vertical or horizontal axis
        private int _numberOfZonesX, _numberOfZonesY, _numberOfZonesZ, _numberOfZonesTotal; //Sizes of the grid
        private int _xGrid, _yGrid, _zGrid, _indiceGrid; //Coordinates and indice of current pixel in grid
        private int _maxIndiceGrid, _numberOfPointsGrid; //Data of the grid
        private int[] _numberOfPointsAddedGrid; //Table containing number of pixels per cube in the grid

        public KinectData(int id, Pose pose, Resolution resolution, Calibration calibration, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax, float cubeSize)
        {
            _id = id;
            _numberOfPoints = 0;
            _position = pose;
            _matrixTransformation = new MatrixTransformation(_position);
            _resolution = resolution;
            _calibration = calibration;
            _focalLength = (float)Math.Sqrt(Math.Pow(_calibration._F_x, 2) + Math.Pow(_calibration._F_y, 2));
            _baseLength = (float)Math.Sqrt(Math.Pow(_calibration._C_x, 2) + Math.Pow(_calibration._C_y, 2));
            _imageBeforeFiltering = null;
            _imageAfterFiltering = null;
            _standardDeviationsBeforeFiltering = null;
            _standardDeviationsAfterFiltering = null;
            _colorValues = new byte[4 * _resolution._x * _resolution._y];
            _depthValues = new DepthImagePixel[_resolution._x * _resolution._y];
            _xMin = xMin; _xMax = xMax;
            _yMin = yMin; _yMax = yMax;
            _zMin = zMin; _zMax = zMax;
            _cubeSize = cubeSize;
            _minAngleHorizontal = (float)-1.0205; _maxAngleHorizontal = (float)1.0205;
            _minAngleVertical = (float)-0.79547; _maxAngleVertical = (float)0.79547;

            _currentNumberOfPoints = 0;
            _currentX = 0; _currentY = 0; _currentZ = 0;
            _currentStandardDeviation = 0;
            _positionPixel = new Vector3();
            _tempImageGetData = new JsonPoint[_resolution._x * _resolution._y];
            _tempStandardDeviationGetData = new float[_resolution._x * _resolution._y];
            _numberOfZonesX = (int)(Math.Ceiling(((_xMax - _xMin) / _cubeSize)));
            _numberOfZonesY = (int)(Math.Ceiling(((_yMax - _yMin) / _cubeSize)));
            _numberOfZonesZ = (int)(Math.Ceiling(((_zMax - _zMin) / _cubeSize)));
            _numberOfZonesTotal = (_numberOfZonesX+10) * (_numberOfZonesY+10) * (_numberOfZonesZ+10);
            _tempPositionsVoxelGrid = new float[3*(_numberOfZonesTotal)];
            _tempColorsVoxelGrid = new int[3*_numberOfZonesTotal];
            _tempStandardDeviationVoxelGrid = new float[_numberOfZonesTotal];
            _xGrid = 0; _yGrid = 0; _zGrid = 0;
            _maxIndiceGrid = 0;
            _numberOfPointsGrid = 0;
            _numberOfPointsAddedGrid = new int[_numberOfZonesTotal];
        }

        public int getId()
        {
            return _id;
        }

        public int getNumberOfPoints()
        {
            return _numberOfPoints;

        }

        public float[] getStandardDeviationBeforeFiltering()
        {
            return _standardDeviationsBeforeFiltering;
        }

        public float[] getStandardDeviationAfterFiltering()
        {
            return _standardDeviationsAfterFiltering;
        }

        public JsonPoint[] getImageBeforeFiltering()
        {
            return _imageBeforeFiltering;
        }

        public JsonPoint[] getImageAfterFiltering()
        {
            return _imageAfterFiltering;
        }

        public Pose getPosition()
        {
            return _position;
        }

        public MatrixTransformation getMatrixTransformation()
        {
            return _matrixTransformation;
        }

        public Resolution getResolution()
        {
            return _resolution;
        }

        public Calibration getCalibration()
        {
            return _calibration;
        }

        public float getFocalLength()
        {
            return _focalLength;
        }

        public float getBaseLength()
        {
            return _baseLength;
        }

        public Byte[] getColorValues()
        {
            return _colorValues;
        }

        public DepthImagePixel[] getDepthValues()
        {
            return _depthValues;
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

        public void updateColorData(ColorImageFrame colorFrame)
        {
            _colorValues = colorFrame.GetRawPixelData();
        }

        public void updateDepthData(DepthImageFrame depthFrame)
        {
            _depthValues = depthFrame.GetRawPixelData();
        }

        public float getMinAngleVertical()
        {
            return _minAngleVertical;
        }

        public float getMaxAngleVertical()
        {
            return _maxAngleVertical;
        }

        public float getMinAngleHorizontal()
        {
            return _minAngleHorizontal;
        }

        public float getMaxAngleHorizontal()
        {
            return _maxAngleHorizontal;
        }

        public void updateImageData()
        {
            //Calculating ranges in Kinect system
            _vectorMin.x = _xMin; _vectorMin.y = _yMin; _vectorMin.z = _zMin;
            _vectorMax.x = _xMax; _vectorMax.y = _yMax; _vectorMax.z = _zMax;
            _currentNumberOfPoints = 0;

            //Getting the data

            for (int i = 0; i < _resolution._x; i++)
            {
                for (int j = 0; j < _resolution._y; j++)
                {
                    //Getting Pixel position in Kinect system
                    _currentX = (float)_depthValues[i + _resolution._x * j].Depth / 1000;
                    _currentY = (float)((2 * i - _calibration._C_x) * _currentX / _calibration._F_x);
                    _currentZ = (float)((2 * j - _calibration._C_y) * _currentX / _calibration._F_y);
                    //Going back to the original system
                    _positionPixel.x = _currentX; _positionPixel.y = _currentY; _positionPixel.z = _currentZ;
                    _positionPixel = _matrixTransformation.multiplyByVector3(_positionPixel);
                    //Console.WriteLine("Positions : {0} {1} {2}", _positionPixel.x, _positionPixel.y, _positionPixel.z);
                    //Console.WriteLine("Minimum : {0} {1} {2}", _vectorMin.x, _vectorMin.y, _vectorMin.z);
                    //Console.WriteLine("Maximum : {0} {1} {2}", _vectorMax.x, _vectorMax.y, _vectorMax.z);
                    if (_currentX != 0)
                    {
                        if (_positionPixel.x > _vectorMin.x && _positionPixel.x < _vectorMax.x && _positionPixel.y > _vectorMin.y && _positionPixel.y < _vectorMax.y && _positionPixel.z > _vectorMin.z && _positionPixel.z < _vectorMax.z)
                        {
                            //Console.WriteLine(_positionPixel.z);
                            //Getting the horizontal and vertical angles
                            _currentAngleVertical = (float)Math.Atan(_currentZ / _currentX);
                            _currentAngleHorizontal = (float)Math.Atan(_currentY / _currentX);
                            if (true/*_currentAngleVertical > _minAngleVertical && _currentAngleVertical < _maxAngleVertical && _currentAngleHorizontal > _minAngleHorizontal && _currentAngleHorizontal < _maxAngleHorizontal*/)
                            {
                                //Getting the standard deviation for the point
                                _currentStandardDeviation = (float)(Math.Sqrt(Math.Pow(_currentAngleHorizontal, 2) + Math.Pow(_currentAngleVertical,2)) * Math.Pow(_currentZ, 2));

                                //Copying the data
                                _tempImageGetData[_currentNumberOfPoints].x = _positionPixel.x;
                                _tempImageGetData[_currentNumberOfPoints].y = _positionPixel.y;
                                _tempImageGetData[_currentNumberOfPoints].z = _positionPixel.z;
                                _tempImageGetData[_currentNumberOfPoints].b = _colorValues[8 * (i + j * 2 * _resolution._x)];
                                _tempImageGetData[_currentNumberOfPoints].g = _colorValues[8 * (i + j * 2 * _resolution._x) + 1];
                                _tempImageGetData[_currentNumberOfPoints].r = _colorValues[8 * (i + j * 2 * _resolution._x) + 2];
                                _tempStandardDeviationGetData[_currentNumberOfPoints] = _currentStandardDeviation;
                                _currentNumberOfPoints++;
                            }
                        }
                    }
                }
            }
            if( _currentNumberOfPoints > 0)
            {
                _numberOfPoints = _currentNumberOfPoints - 1;
            }else
            {
                _numberOfPoints = _currentNumberOfPoints;
            }
            _imageBeforeFiltering = new JsonPoint[_numberOfPoints];
            _standardDeviationsBeforeFiltering = new float[_numberOfPoints];
            for (int i = 0; i < _numberOfPoints; i++)
            {
                _imageBeforeFiltering[i] = _tempImageGetData[i];
                _standardDeviationsBeforeFiltering[i] = _tempStandardDeviationGetData[i];
            }

            for(int i = 0; i < _tempImageGetData.Length; i++)
            {
                _tempImageGetData[i].x = 0; _tempImageGetData[i].r = 0;
                _tempImageGetData[i].y = 0; _tempImageGetData[i].g = 0;
                _tempImageGetData[i].z = 0; _tempImageGetData[i].b = 0;
                _tempStandardDeviationGetData[i] = 0;

            }
        }

        public void voxelGrid()
        //Downsamples the data from _image, with cubes of size _cubeSize
        {
            _maxIndiceGrid = 0;
            for (int i = 0; i < _numberOfPoints; i++)
            {
                _xGrid = (int)Math.Floor((_imageBeforeFiltering[i].x - _xMin) / _cubeSize);
                _yGrid = (int)Math.Floor((_imageBeforeFiltering[i].y - _yMin) / _cubeSize);
                _zGrid = (int)Math.Floor((_imageBeforeFiltering[i].z - _zMin) / _cubeSize);
                if (_xGrid == _numberOfZonesX)
                {
                    _xGrid--;
                }
                if (_yGrid == _numberOfZonesY)
                {
                    _yGrid--;
                }

                if (_zGrid == _numberOfZonesZ)
                {
                    _zGrid--;
                }
                _indiceGrid = ((int)_xGrid + (int)(_numberOfZonesX * _yGrid) + (int)((_numberOfZonesX * _numberOfZonesY) * _zGrid));
                if (_indiceGrid > _maxIndiceGrid)
                {
                    _maxIndiceGrid = _indiceGrid;
                }
                _tempPositionsVoxelGrid[3*_indiceGrid] = _tempPositionsVoxelGrid[3*_indiceGrid] + _imageBeforeFiltering[i].x;
                _tempPositionsVoxelGrid[3*_indiceGrid+1] = _tempPositionsVoxelGrid[3*_indiceGrid+1] + _imageBeforeFiltering[i].y;
                _tempPositionsVoxelGrid[3*_indiceGrid+2] = _tempPositionsVoxelGrid[3*_indiceGrid+2] + _imageBeforeFiltering[i].z;
                _tempColorsVoxelGrid[3*_indiceGrid] = (_tempColorsVoxelGrid[3*_indiceGrid] + _imageBeforeFiltering[i].r);
                _tempColorsVoxelGrid[3*_indiceGrid+1] = (_tempColorsVoxelGrid[3*_indiceGrid+1] + _imageBeforeFiltering[i].g);
                _tempColorsVoxelGrid[3*_indiceGrid+2] = (_tempColorsVoxelGrid[3*_indiceGrid+2] + _imageBeforeFiltering[i].b);
                _tempStandardDeviationVoxelGrid[_indiceGrid] = _tempStandardDeviationVoxelGrid[_indiceGrid] + _standardDeviationsBeforeFiltering[i];
                _numberOfPointsAddedGrid[_indiceGrid] = _numberOfPointsAddedGrid[_indiceGrid] + 1;
            }

            _numberOfPointsGrid = 0;
            for (int i = 0; i < _numberOfZonesTotal; i++)
            {
                if (_numberOfPointsAddedGrid[i] > 0)
                {
                    //Console.WriteLine(_numberOfZonesTotal);
                    _tempPositionsVoxelGrid[3*i] = _tempPositionsVoxelGrid[3*i] / _numberOfPointsAddedGrid[i];
                    _tempPositionsVoxelGrid[3*i+1] = _tempPositionsVoxelGrid[3*i+1] / _numberOfPointsAddedGrid[i];
                    _tempPositionsVoxelGrid[3*i+2] = _tempPositionsVoxelGrid[3*i+2] / _numberOfPointsAddedGrid[i];
                    _tempColorsVoxelGrid[3*i] = (_tempColorsVoxelGrid[3*i] / _numberOfPointsAddedGrid[i]);
                    _tempColorsVoxelGrid[3*i+1] = (_tempColorsVoxelGrid[3*i+1] / _numberOfPointsAddedGrid[i]);
                    _tempColorsVoxelGrid[3*i+2] = (_tempColorsVoxelGrid[3*i+2] / _numberOfPointsAddedGrid[i]);
                    _tempStandardDeviationVoxelGrid[i] = _tempStandardDeviationVoxelGrid[i] / _numberOfPointsAddedGrid[i];
                    _numberOfPointsGrid++;
                }

            }

            if (_numberOfPointsGrid > 0)
            {
                _numberOfPoints = _numberOfPointsGrid;
            }else
            {
                _numberOfPoints = _numberOfPointsGrid;
            }

            _imageAfterFiltering = new JsonPoint[_numberOfPoints];
            _standardDeviationsAfterFiltering = new float[_numberOfPoints];
            int _cpt = 0;
            for (int i = 0; i < _numberOfZonesTotal; i++)
            {
                if (_numberOfPointsAddedGrid[i] > 0)
                {
                    _imageAfterFiltering[_cpt].x = _tempPositionsVoxelGrid[3*i];
                    _imageAfterFiltering[_cpt].y = _tempPositionsVoxelGrid[3 * i+1];
                    _imageAfterFiltering[_cpt].z = _tempPositionsVoxelGrid[3 * i+2];
                    _imageAfterFiltering[_cpt].r = (byte)_tempColorsVoxelGrid[3 * i];
                    _imageAfterFiltering[_cpt].g = (byte)_tempColorsVoxelGrid[3 * i+1];
                    _imageAfterFiltering[_cpt].b = (byte)_tempColorsVoxelGrid[3 * i+2];
                    _standardDeviationsAfterFiltering[_cpt] = _tempStandardDeviationVoxelGrid[i];
                    _tempPositionsVoxelGrid[3*i] = 0;
                    _tempPositionsVoxelGrid[3*i+1] = 0;
                    _tempPositionsVoxelGrid[3*i+2] = 0;
                    _tempColorsVoxelGrid[3*i] = 0;
                    _tempColorsVoxelGrid[3*i+1] = 0;
                    _tempColorsVoxelGrid[3*i+2] = 0;
                    _numberOfPointsAddedGrid[i] = 0;
                    _cpt++;
                }
                
            }
        }

        public void processing()
        //Updates and filters the data
        {
            updateImageData();
            voxelGrid();
        }

        public JsonDepth ToJsonFormat()
        //Converts to a sendable format
        {
            var dataToSend = new JsonDepth()
            {
                Id = _id,
                NbPoints = _numberOfPoints,
                PointCloud = _imageAfterFiltering
            };
            return dataToSend;
        }
    }
}
