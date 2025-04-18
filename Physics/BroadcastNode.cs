using Raylib_cs;
using Physics550Engine_Raylib.Physics.Interfaces;
using System.Numerics;
using Microsoft.VisualBasic.FileIO;

namespace Physics550Engine_Raylib.Physics
{

    public class BroadcastNode : INode
    {

        public List<INode> Nodes = new List<INode>();

        public bool IsDebugDrawn { get; set; }

        public BroadcastNode()
        {
            IsDebugDrawn = false;
        }

        public void AddNode(INode node)
        {
            Nodes.Add(node);
        }

        public void Step()
        {
            foreach (var node in Nodes)
            {
                node.Step();
            }
        }
        public void Draw()
        {
            foreach (var node in Nodes)
            {
                node.Draw();
            }
        }

        public void Clear()
        {
            foreach (var node in Nodes)
            {
                node.Clear();
            }
            Nodes.Clear();
        }
    }
}
