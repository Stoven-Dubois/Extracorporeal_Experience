using Demonixis.Kinect.Server;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Demonixis.Server.Kinect
{
    // Pose with position (x,y,z) and rotation (Euler angles - ZYX)
    public struct Pose
    {
        public Vector3 _position { get; set; }
        public Vector3 _rotation { get; set; }

        public Pose(Vector3 position, Vector3 rotation)
        {
            _position = position;
            _rotation = rotation;
        }
    }
        // Class containing the transformation tools to transfer 3D points through different axis systems
        class MatrixTransformation
        {
            //Accessible data
            private float[] _matrix; //Matrix of transformation from original to transformed axis systems
            private float[] _inverseMatrix; //Matrix of transfrom from transformed to original axis system
            private float _det; //Determinant of _matrix

            //Non-Accessible data
            private Vector3 _resultMultiplyByVector3, _resultMultiplyInverseByVector3; //3D-Point resulting from transformations

            public float[] InitializeMatrix(float[] matrix)
            //Makes Matrix Identity
            {
                matrix = new float[16];
                matrix[0] = 1; matrix[1] = 0; matrix[2] = 0; matrix[3] = 0;
                matrix[4] = 0; matrix[5] = 1; matrix[6] = 0; matrix[7] = 0;
                matrix[8] = 0; matrix[9] = 0; matrix[10] = 1; matrix[11] = 0;
                matrix[12] = 0; matrix[13] = 0; matrix[14] = 0; matrix[15] = 1;
                return matrix;
            }

            public MatrixTransformation()
            //Constructor with no defined pose
            {
                InitializeMatrix(_matrix);
                InitializeMatrix(_inverseMatrix);
                _det = 1;
                _resultMultiplyByVector3 = new Vector3();
                _resultMultiplyInverseByVector3 = new Vector3();
            }

            public MatrixTransformation(Pose pose)
            //Constructeur with defined Pose
            {
                _matrix = InitializeMatrix(_matrix);
                _inverseMatrix = InitializeMatrix(_inverseMatrix);
                _det = 1;
                rotate(pose._rotation.x, 3);
                rotate(pose._rotation.y, 2);
                rotate(pose._rotation.z, 1);
                translate(pose._position);
                printMatrix(_matrix, 4, 4);
                updateInverseMatrix();
                //printMatrix(_inverseMatrix, 4, 4);
            }

        public void printMatrix(float[] matrix, int sizeX, int sizeY)
            {
                Console.WriteLine("Showing Matrix : ");
                for(int i = 0; i < sizeX; i++)
                {
                    for(int j=0; j< sizeY; j++)
                    {
                        Console.Write("{0} ", matrix[j + sizeY * i]);
                    }
                    Console.Write("\n");
                } 
            }
               

            public float[] getMatrix()
            {
                return _matrix;
            }

            public float[] getInverseMatrix()
            {
                return _inverseMatrix;
            }

            public float getDeterminant()
            {
                return _det;
            }

            public Vector3 multiplyByVector3(Vector3 vector)
            {
            //Gets the result from the multiplication of _matrix*vector
                _resultMultiplyByVector3.x = _matrix[0] * vector.x + _matrix[1] * vector.y + _matrix[2] * vector.z + _matrix[3];
                _resultMultiplyByVector3.y = _matrix[4] * vector.x + _matrix[5] * vector.y + _matrix[6] * vector.z + _matrix[7];
                _resultMultiplyByVector3.z = _matrix[8] * vector.x + _matrix[9] * vector.y + _matrix[10] * vector.z + _matrix[11];
                return (_resultMultiplyByVector3);
            }

            public Vector3 multiplyInverseByVector3(Vector3 vector)
            {
                //Gets the result from the multiplication of _inverseMatrix*vector
                _resultMultiplyInverseByVector3.x = _inverseMatrix[0] * vector.x + _inverseMatrix[1] * vector.y + _inverseMatrix[2] * vector.z + _inverseMatrix[3];
                _resultMultiplyInverseByVector3.y = _inverseMatrix[4] * vector.x + _inverseMatrix[5] * vector.y + _inverseMatrix[6] * vector.z + _inverseMatrix[7];
                _resultMultiplyInverseByVector3.z = _inverseMatrix[8] * vector.x + _inverseMatrix[9] * vector.y + _inverseMatrix[10] * vector.z + _inverseMatrix[11];
                return (_resultMultiplyInverseByVector3);
            }

            public void updateInverseMatrix()
            {
            //Updates _inverseMatrix depending on _matrix
                _det = _matrix[0] * (_matrix[5] * _matrix[10] - _matrix[6] * _matrix[9]) - _matrix[4] * (_matrix[1] * _matrix[10] - _matrix[2] * _matrix[9]) + _matrix[8] * (_matrix[1] * _matrix[6] - _matrix[2] * _matrix[5]);
                _inverseMatrix[0] = (_matrix[5] * _matrix[10] - _matrix[6] * _matrix[9]) / _det;
                _inverseMatrix[4] = -(_matrix[4] * _matrix[10] - _matrix[6] * _matrix[8]) / _det;
                _inverseMatrix[8] = (_matrix[4] * _matrix[9] - _matrix[8] * _matrix[5]) / _det;
                _inverseMatrix[1] = -(_matrix[1] * _matrix[10] - _matrix[9] * _matrix[2]) / _det;
                _inverseMatrix[5] = (_matrix[0] * _matrix[10] - _matrix[2] * _matrix[8]) / _det;
                _inverseMatrix[9] = -(_matrix[0] * _matrix[9] - _matrix[8] * _matrix[1]) / _det;
                _inverseMatrix[2] = (_matrix[1] * _matrix[6] - _matrix[5] * _matrix[2]) / _det;
                _inverseMatrix[6] = -(_matrix[0] * _matrix[6] - _matrix[4] * _matrix[2]) / _det;
                _inverseMatrix[10] = (_matrix[5] * _matrix[0] - _matrix[4] * _matrix[1]) / _det;

                _inverseMatrix[3] = -_matrix[3];
                _inverseMatrix[7] = -_matrix[7];
                _inverseMatrix[11] = -_matrix[11];
            }

            public void translate(Vector3 position)
            //Translates _matrix of position vector
            {
                _matrix[3] = _matrix[3] + position.x;
                _matrix[7] = _matrix[7] + position.y;
                _matrix[11] = _matrix[11] + position.z;
            }

            public void rotate(float angle, int axis)
            //Rotates _date of angle around axis, axis = 1 for x, 2 for y, 3 for z
            {
            float[] dataRotation = new float[9];
                if (axis == 1)
                {
                    dataRotation[0] = 1; dataRotation[1] = 0; dataRotation[2] = 0;
                    dataRotation[3] = 0; dataRotation[4] = (float)Math.Cos(angle); dataRotation[5] = (float)-Math.Sin(angle);
                    dataRotation[6] = 0; dataRotation[7] = (float)Math.Sin(angle); dataRotation[8] = (float)Math.Cos(angle);
                }
                else if (axis == 2)
                {
                    dataRotation[0] = (float)Math.Cos(angle); dataRotation[1] = 0; dataRotation[2] = (float)Math.Sin(angle);
                    dataRotation[3] = 0; dataRotation[4] = 1; dataRotation[5] = 0;
                    dataRotation[6] = -(float)Math.Sin(angle); dataRotation[7] = 0; dataRotation[8] = (float)Math.Cos(angle);
                }
                else if (axis == 3)
                {
                    dataRotation[0] = (float)Math.Cos(angle); dataRotation[1] = (float)-Math.Sin(angle); dataRotation[2] = 0;
                    dataRotation[3] = (float)Math.Sin(angle); dataRotation[4] = (float)Math.Cos(angle); dataRotation[5] = 0;
                    dataRotation[6] = 0; dataRotation[7] = 0; dataRotation[8] = 1;
                }

            float[] result = { 0, 0, 0, _matrix[3], 0, 0, 0, _matrix[7], 0, 0, 0, _matrix[11], 0, 0, 0, _matrix[15] };
                for (int k = 0; k < 3; k++)
                {
                    for (int l = 0; l < 3; l++)
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            result[4 * k + l] = result[4 * k + l] + _matrix[4 * k + i] * dataRotation[3 * i + l];
                        }
                    }
                }
                _matrix = result;
            }
        }
    }
