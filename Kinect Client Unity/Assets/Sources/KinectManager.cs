using UnityEngine;
using WebSocketSharp;
using Newtonsoft.Json;
using System;

namespace Demonixis.Kinect
{
    public struct UserPosition
    {
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
    }

    public sealed class KinectManager : MonoBehaviour
    {
        #region Constants and Static Fields

        public const int KinectXbox360JointCount = 20;
        public const int KinectXboxOneJointCount = 25;
        private static KinectManager _instance = null;
        public GameObject prefab;
        private static 
        #endregion

        #region Private Variables

        WebSocket _webSocket = null;
        private Body[] _bodies;
        private JsonDepth _Image;
        private int _bodyCount = 0;
        private UserPosition[] _usersTransform;
        private Joint _dummyJoint = new Joint();
        private Joint _spinBaseJoint = new Joint();
        private Mesh _mesh;
        private int _numberMaximumOfPoints;
        private Vector3[] _vert;
        private Color32[] _colors32;
        private int[] _tris;
        private int[] _tempTris;
        private int _indexNeighbour1, _indexNeighbour2;


        #endregion

        #region Editor Fields

        [Header("Server Settings")]
        [SerializeField]
        private string _ip = "127.0.0.1";
        [SerializeField]
        private string _port = "8181";
        [SerializeField ]
        private bool _showLog = true;
        [SerializeField]
        private bool _autoConnect = true;

        #endregion

        #region Properties

        public static KinectManager Instance
        {
            get
            {
                if (_instance == null)
                    _instance = FindObjectOfType<KinectManager>();

                return _instance;
            }
        }

        public bool IsInitialized
        {
            get; private set;
        }

        public Body[] Bodies
        {
            get { return _bodies; }
        }

        public int KinectVersion
        {
            get { return _bodyCount > 0 ? _bodies[0].Version : 0; }
        }

        public bool IsKinectV1
        {
            get { return _bodyCount > 0 ? _bodies[0].Version == 1 : false; }
        }

        public bool IsKinectV2
        {
            get { return _bodyCount > 0 ? _bodies[0].Version == 2 : false; }
        }

        public bool IsConnected
        {
            get { return _bodyCount > 0; }
        }

        #endregion

        #region Lifecycle

        void Awake()
        {
            if (_instance != null)
            {
                Debug.Log("[KinectManager] Multiple instances is not supported.");
                Debug.Log(string.Format("[KinectManager] The object attached to {0} will be destroyed", gameObject.name));
                Destroy(this);
            }

            _instance = this;

            if (_autoConnect)
            {
                Debug.Log("[KinectManager] Trying to connect...");
                Connect(_ip, _port);
            }

            //Starting the mesh
            _mesh = new Mesh();
            if (!GetComponent<MeshFilter>())
            {
                gameObject.AddComponent<MeshFilter>();
            }
            if (!GetComponent<MeshRenderer>())
            {
                gameObject.AddComponent<MeshRenderer>();
            }
            gameObject.GetComponent<MeshFilter>().mesh = _mesh;

            _numberMaximumOfPoints = 4000;
            _tempTris = new int[325 * 3 * _numberMaximumOfPoints];
        }

        void OnDestroy()
        {
            if (_webSocket != null)
            {
                _webSocket.OnMessage -= OnMessage;
                _webSocket.Close();
            }
        }

        public void Connect()
        {
            Connect(_ip, _port);
        }

        public void Connect(string ip, string port)
        {
            if (_webSocket != null)
            {
                _webSocket.OnMessage -= OnMessage;
                _webSocket.Close();
                _webSocket = null;
            }

            _webSocket = new WebSocket(string.Format("ws://{0}:{1}", _ip, _port));

            if (_showLog)
                _webSocket.OnOpen += (s, e) => Debug.Log("[KinectManager] Connected to the Kinect Server.");

            if (_showLog)
                _webSocket.OnError += (s, e) => Debug.LogFormat("[KinectManager] An Error was detected.\n{0}", e.Message);

            _webSocket.Connect();
            _webSocket.OnMessage += OnMessage;
        }

        #endregion

        #region Methods to get data on a body

        public Joint GetJoint(JointType type, int userIndex)
        {
            if (_bodyCount > userIndex)
            {
                var index = (int)type;
                if (IsKinectV2 || index < KinectXbox360JointCount)
                    return _bodies[userIndex].Joints[index];
            }

            return _dummyJoint;
        }

        public Body GetBody(int userIndex)
        {
            if (_bodyCount > userIndex)
                return _bodies[userIndex];

            return null;
        }

        public Vector3 GetJointPosition(JointType type, int userIndex)
        {
            return GetJoint(type, userIndex).Position;
        }

        public Quaternion GetJointOrientation(JointType type, int userIndex)
        {
            return GetJoint(type, userIndex).Rotation;
        }

        public Vector3 GetUserPosition(int userIndex)
        {
            if (_bodyCount > userIndex)
                return _usersTransform[userIndex].Position;

            return Vector3.zero;
        }

        public Quaternion GetUserRotation(int userIndex)
        {
            if (_bodyCount > userIndex)
                return _usersTransform[userIndex].Rotation;

            return Quaternion.identity;
        }

        public HandState GetHandState(bool handLeft, int userIndex)
        {
            if (_bodyCount > userIndex)
                return handLeft ? _bodies[userIndex].HandLeftState : _bodies[userIndex].HandRightState;
            else
                return HandState.Unknown;
        }

        public TrackingConfidence GetHandConfidence(bool handLeft, int userIndex)
        {
            if (_bodyCount > userIndex)
                return handLeft ? _bodies[userIndex].HandLeftConfidence : _bodies[userIndex].HandRightConfidence;
            else
                return TrackingConfidence.Low;
        }

        public Joint GetParentJoint(JointType type, int userIndex)
        {
            Joint parent = null;

            switch (type)
            {
                case JointType.SpineMid:
                case JointType.HipLeft:
                case JointType.HipRight:
                case JointType.KneeLeft:
                case JointType.KneeRight:
                case JointType.AnkleLeft:
                case JointType.AnkleRight:
                case JointType.FootLeft:
                case JointType.FootRight:
                    parent = GetJoint(JointType.SpineBase, userIndex);
                    break;

                case JointType.SpineShoulder:
                    parent = GetJoint(JointType.SpineMid, userIndex);
                    break;

                case JointType.Head:
                case JointType.Neck:
                case JointType.ShoulderLeft:
                case JointType.ShoulderRight:
                case JointType.ElbowLeft:
                case JointType.ElbowRight:
                case JointType.WristLeft:
                case JointType.WristRight:
                    parent = GetJoint(JointType.SpineShoulder, userIndex);
                    break;

                case JointType.HandRight:
                case JointType.ThumbRight:
                    parent = GetJoint(JointType.WristRight, userIndex);
                    break;

                case JointType.HandTipRight:
                    parent = GetJoint(JointType.HandRight, userIndex);
                    break;

                case JointType.HandLeft:
                case JointType.ThumbLeft:
                    parent = GetJoint(JointType.WristLeft, userIndex);
                    break;

                case JointType.HandTipLeft:
                    parent = GetJoint(JointType.HandLeft, userIndex);
                    break;
            }

            return parent;
        }

        #endregion

        #region WebSocket Handler

        private void OnMessage(object sender, MessageEventArgs e)
        {
            _Image = JsonConvert.DeserializeObject<JsonDepth>(e.Data);
        }


        #endregion

        private void FixedUpdate()
        {
            float xt, yt, zt;
            if (_Image != null)
            {
                JsonDepth currentFrame = _Image;
                _Image = null;
                if (!IsInitialized)
                    IsInitialized = true;
                if (currentFrame.NbPoints > 0)
                {
                    //Debug.Log("[KinectManager] Nombre de points reçu : " + currentFrame.NbPoints + " de la Kinect ID : " + currentFrame.Id);
                }

                int _numberOfTriangles = 0;

                _vert = new Vector3[currentFrame.NbPoints];
                _colors32 = new Color32[currentFrame.NbPoints];

                for (int i = 0; i < currentFrame.NbPoints; i++)
                {
                    _vert[i].x = currentFrame.PointCloud[i].x;
                    _vert[i].y = currentFrame.PointCloud[i].y;
                    _vert[i].z = currentFrame.PointCloud[i].z;
                    _colors32[i] = new Color32((currentFrame.PointCloud[i].r), (currentFrame.PointCloud[i].g),(currentFrame.PointCloud[i].b), 1);
                }


                int _currentNeighbour, _currentNeighbourNeighbour;

                _numberOfTriangles = 0;
                for(int i = 0; i < currentFrame.NbPoints; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        for (int k = -1; k <= 1; k++)
                        {
                            for (int l = -1; l <= 1; l++)
                            {
                                _currentNeighbour = 9 * (j + 1) + 3 * (k + 1) + (l + 1);
                                if (_currentNeighbour>=0 && _currentNeighbour<27 && (j != 0 || k != 0 || l != 0) && currentFrame.PointCloud[i].neighbours[_currentNeighbour] > 0)
                                {
                                    for (int jj = j-1; jj <= j + 1; jj++)
                                    {
                                        for (int kk = k-1; kk <= k + 1; kk++)
                                        {
                                            for (int ll = l-1; ll <= l + 1; ll++)
                                            {
                                                _currentNeighbourNeighbour = 9 * (jj + 1) + 3 * (kk + 1) + (ll + 1);
                                                if (_currentNeighbourNeighbour >= 0 && _currentNeighbourNeighbour < 27 && (jj != j || kk != k || ll != l) && currentFrame.PointCloud[i].neighbours[_currentNeighbourNeighbour] > 0)
                                                {
                                                    _indexNeighbour1 = currentFrame.PointCloud[i].neighbours[_currentNeighbour];
                                                    _indexNeighbour2 = currentFrame.PointCloud[i].neighbours[_currentNeighbourNeighbour];
                                                    _tempTris[3 * _numberOfTriangles] = i;
                                                    _tempTris[3 * _numberOfTriangles + 1] = _indexNeighbour1;
                                                    _tempTris[3 * _numberOfTriangles + 2] = _indexNeighbour2;
                                                    _numberOfTriangles++;
                                                }
                                            }
                                        }
                                    } 
                                }
                            }
                        }
                    }
                }

                _tris = new int[3 * _numberOfTriangles];
                for (int i = 0; i < 3 * _numberOfTriangles; i++)
                {
                    _tris[i] = _tempTris[i];
                }
                
                Debug.Log("Creating Mesh : " + _numberOfTriangles);

                _mesh.Clear();
                _mesh.vertices = _vert;
                _mesh.triangles = _tris;
                _mesh.RecalculateNormals();
                _mesh.colors32 = _colors32;

                gameObject.GetComponent<MeshFilter>().mesh = _mesh;


                //int sumColor = currentFrame.PointCloud[i].r + currentFrame.PointCloud[i].g + currentFrame.PointCloud[i].b;
                //point.GetComponent<MeshRenderer>().material.color = new Color32(currentFrame.PointCloud[i].b, currentFrame.PointCloud[i].g, currentFrame.PointCloud[i].r, 255);
            }
        }
    }
}