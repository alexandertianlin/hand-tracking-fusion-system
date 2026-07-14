using UnityEngine;

public class HandAntiClipping : MonoBehaviour
{
    [Header("防穿模图层配置")]
    [Tooltip("专门给手套骨骼设置的物理图层，记得在Project Settings里面让这个图层自己与自己能发生碰撞")]
    public LayerMask gloveLayer;

    [Header("矫正平滑度")]
    [Range(5f, 20f)] public float stiffness = 12f;

    private HandMotionManager motionManager;
    private Collider[] allColliders;

    // 🔥 预分配物理缓冲池，终结 OverlapBox 带来的 GC Allocation
    private Collider[] overlapBuffer = new Collider[10];

    void Start()
    {
        motionManager = GetComponent<HandMotionManager>();
        allColliders = GetComponentsInChildren<Collider>();

        foreach (var col in allColliders)
        {
            col.isTrigger = true;
            if (!col.gameObject.GetComponent<Rigidbody>())
            {
                var rb = col.gameObject.AddComponent<Rigidbody>();
                rb.isKinematic = true;
            }
        }
    }

    void LateUpdate()
    {
        if (allColliders == null || allColliders.Length == 0) return;

        for (int iteration = 0; iteration < 2; iteration++)
        {
            foreach (var colA in allColliders)
            {
                if (!colA.enabled) continue;

                // 🔥 换用 NonAlloc 接口，复用 overlapBuffer 数组，做到内存零分配
                int hits = Physics.OverlapBoxNonAlloc(colA.bounds.center, colA.bounds.extents, overlapBuffer, colA.transform.rotation, gloveLayer);

                for (int i = 0; i < hits; i++)
                {
                    Collider colB = overlapBuffer[i];
                    if (colA == colB) continue;

                    if (Physics.ComputePenetration(
                        colA, colA.transform.position, colA.transform.rotation,
                        colB, colB.transform.position, colB.transform.rotation,
                        out Vector3 direction, out float distance))
                    {
                        if (distance > 0.001f)
                        {
                            ResolveBoneClipping(colA, direction, distance);
                        }
                    }
                }
            }
        }
    }

    private void ResolveBoneClipping(Collider boneCollider, Vector3 pushDirection, float distance)
    {
        Transform boneTex = boneCollider.transform;
        Vector3 localPushDir = boneTex.InverseTransformDirection(pushDirection);
        float angleCorrection = distance * Mathf.Rad2Deg * stiffness;

        if (Vector3.Dot(localPushDir, Vector3.up) > 0f || Vector3.Dot(localPushDir, Vector3.forward) > 0f)
        {
            boneTex.localRotation = Quaternion.AngleAxis(-angleCorrection, motionManager.fingerBendAxis) * boneTex.localRotation;
        }
    }
}