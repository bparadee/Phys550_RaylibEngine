using Raylib_cs;
using Physics550Engine_Raylib.Physics.Interfaces;
using System.Numerics;

namespace Physics550Engine_Raylib.Physics
{

    public class BroadcastNode : INode
    {
        public event EventHandler<EventArgs>? DrawEvent;
        public event EventHandler<EventArgs>? StepEvent;

        public bool IsDebugDrawn { get; set; }

        public BroadcastNode()
        {
            IsDebugDrawn = false;
        }

        public void AddNode(INode node)
        {
            DrawEvent += node.OnDraw;
            StepEvent += node.OnStep;
        }

        public void Step()
        { 
            EventArgs args = new();
            StepEvent?.Invoke(this, args);
        }
        public void Draw()
        {
            EventArgs args = new();
            DrawEvent?.Invoke(this, args);
        }

        public void OnStep(object? _, EventArgs __)
        {
            Step();
        }

        public void OnDraw(object? _, EventArgs __)
        {
            Draw();
        }
    }
}
