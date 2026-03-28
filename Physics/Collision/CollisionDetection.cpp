#include "CollisionDetection.h"
#include "../Core/PhysicsConstants.h"

#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>

/** @brief Finds the point on line segment AB closest to point P. */
static Vec2 ClosestPointOnLineSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    Vec2 ap = p - a;
    float proj = ap.Dot(ab);
    float distanceSq = ab.MagnitudeSquared();

    if (distanceSq <= mage::math::EPSILON_SQ) return a;

    float t = std::clamp(proj / distanceSq, 0.0f, 1.0f);
    return a + ab * t;
}

/** @brief Finds the closest points between two line segments (P1-Q1) and (P2-Q2). */
static void ClosestPointsOnTwoLineSegments(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2, Vec2& closest1, Vec2& closest2) {
    Vec2 d1 = q1 - p1;
    Vec2 d2 = q2 - p2;
    Vec2 r = p1 - p2;

    float a = d1.Dot(d1);
    float e = d2.Dot(d2);
    float f = d2.Dot(r);

    float s = 0.0f; // Parameter [0,1] for segment 1
    float t = 0.0f; // Parameter [0,1] for segment 2

    if (a <= mage::math::EPSILON_SQ && e <= mage::math::EPSILON_SQ) {
        closest1 = p1; closest2 = p2;
        return;
    }

    if (a <= mage::math::EPSILON_SQ) {
        s = 0.0f;
        t = std::clamp(f / e, 0.0f, 1.0f);
    }
    else if (e <= mage::math::EPSILON_SQ) {
        t = 0.0f;
        s = std::clamp(-d1.Dot(r) / a, 0.0f, 1.0f);
    }
    else {
        float b = d1.Dot(d2);
        float c = d1.Dot(r);
        float denom = a * e - b * b;

        s = (denom > mage::math::EPSILON) ? std::clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
        t = (b * s + f) / e;

        if (t < 0.0f) {
            t = 0.0f;
            s = std::clamp(-c / a, 0.0f, 1.0f);
        }
        else if (t > 1.0f) {
            t = 1.0f;
            s = std::clamp((b - c) / a, 0.0f, 1.0f);
        }
    }

    closest1 = p1 + d1 * s;
    closest2 = p2 + d2 * t;
}

bool CollisionDetection::IsColliding(Body* a, Body* b, std::vector<Contact>& contacts) {
    ShapeType typeA = a->shape->GetType();
    ShapeType typeB = b->shape->GetType();

    bool aIsCircle = (typeA == CIRCLE);
    bool bIsCircle = (typeB == CIRCLE);
    bool aIsCapsule = (typeA == CAPSULE);
    bool bIsCapsule = (typeB == CAPSULE);
    bool aIsPolygon = (typeA == POLYGON || typeA == BOX);
    bool bIsPolygon = (typeB == POLYGON || typeB == BOX);

    // 1. Same-Shape Collisions
    if (aIsCircle && bIsCircle)   return IsCollidingCircleCircle(a, b, contacts);
    if (aIsPolygon && bIsPolygon) return IsCollidingPolygonPolygon(a, b, contacts);
    if (aIsCapsule && bIsCapsule) return IsCollidingCapsuleCapsule(a, b, contacts);

    // 2. Polygon vs Circle
    if (aIsPolygon && bIsCircle)  return IsCollidingPolygonCircle(a, b, contacts);
    if (aIsCircle && bIsPolygon)  return IsCollidingPolygonCircle(b, a, contacts);

    // 3. Capsule vs Circle
    if (aIsCapsule && bIsCircle)  return IsCollidingCapsuleCircle(a, b, contacts);
    if (aIsCircle && bIsCapsule)  return IsCollidingCapsuleCircle(b, a, contacts);

    // 4. Capsule vs Polygon
    if (aIsCapsule && bIsPolygon) return IsCollidingCapsulePolygon(a, b, contacts);
    if (aIsPolygon && bIsCapsule) return IsCollidingCapsulePolygon(b, a, contacts);

    return false;
}

bool CollisionDetection::IsCollidingCircleCircle(Body* a, Body* b, std::vector<Contact>& contacts) {
    CircleShape* aCircleShape = static_cast<CircleShape*>(a->shape.get());
    CircleShape* bCircleShape = static_cast<CircleShape*>(b->shape.get());

    const Vec2 ab = b->position - a->position;
    const float radiusSum = aCircleShape->radius + bCircleShape->radius;

    bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);

    if (!isColliding) return false;
    
    Contact contact;
    contact.a = a;
    contact.b = b;

    contact.normal = ab;
    contact.normal.Normalize();

    contact.start = b->position - contact.normal * bCircleShape->radius;
    contact.end = a->position + contact.normal * aCircleShape->radius;

    contact.depth = (contact.end - contact.start).Magnitude();

    contacts.push_back(contact);

    return true;
}

bool CollisionDetection::IsCollidingCapsuleCircle(Body* capsule, Body* circle, std::vector<Contact>& contacts) {
    const auto* cap = static_cast<const CapsuleShape*>(capsule->shape.get());
    const auto* circ = static_cast<const CircleShape*>(circle->shape.get());

    const Vec2& center = circle->position;

    // Closest point on capsule spine
    Vec2 closest = ClosestPointOnLineSegment(center, cap->worldP1, cap->worldP2);

    Vec2 diff = center - closest;
    float distSq = diff.MagnitudeSquared();

    float radiusSum = cap->radius + circ->radius;
    float radiusSumSq = radiusSum * radiusSum;

    // Early out
    if (distSq > radiusSumSq) return false;

    float dist = std::sqrt(distSq);

    Vec2 normal;
    if (dist > mage::math::EPSILON) {
        float invDist = 1.0f / dist;
        normal = diff * invDist;
    }
    else {
        normal = Vec2(0.0f, 1.0f);
    }

    Contact contact;
    contact.a = capsule;
    contact.b = circle;
    contact.normal = normal;
    contact.depth = radiusSum - dist;

    contact.start = closest + normal * cap->radius;
    contact.end = contact.start + normal * contact.depth;

    contacts.push_back(contact);
    return true;
}

bool CollisionDetection::IsCollidingCapsuleCapsule(Body* a, Body* b, std::vector<Contact>& contacts) {
    const auto* capA = static_cast<CapsuleShape*>(a->shape.get());
    const auto* capB = static_cast<CapsuleShape*>(b->shape.get());

    Vec2 closestA, closestB;
    ClosestPointsOnTwoLineSegments(capA->worldP1, capA->worldP2, capB->worldP1, capB->worldP2, closestA, closestB);

    Vec2 diff = closestB - closestA;
    float distSq = diff.MagnitudeSquared();
    float radiusSum = capA->radius + capB->radius;

    if (distSq > radiusSum * radiusSum) return false;

    float dist = std::sqrt(distSq);
    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.depth = radiusSum - dist;
    contact.normal = (dist > mage::math::EPSILON) ? (diff / dist) : Vec2(0, 1);

    contact.start = closestA + contact.normal * capA->radius;
    contact.end = contact.start + contact.normal * contact.depth;

    contacts.push_back(contact);
    return true;
}

bool CollisionDetection::IsCollidingCapsulePolygon(Body* capsule, Body* poly, std::vector<Contact>& contacts) {
    const CapsuleShape* capShape = static_cast<const CapsuleShape*>(capsule->shape.get());
    const PolygonShape* polyShape = static_cast<const PolygonShape*>(poly->shape.get());

    const float radius = capShape->radius;
    // Central segment already in world space
    const Vec2& capA = capShape->worldP1;
    const Vec2& capB = capShape->worldP2;
    const auto& verts = polyShape->worldVertices;
    const int n = static_cast<int>(verts.size());

    // Variables to track the deepest penetration
    float maxDepth = 0.0f;
    Vec2 bestNormal(0.0f, 0.0f);
    Vec2 bestOnEdge(0.0f, 0.0f);
    Vec2 bestOnCapsule(0.0f, 0.0f);
    Vec2 bestEdgeA(0.0f, 0.0f);
    Vec2 bestEdgeB(0.0f, 0.0f);
    bool found = false;

    // Find the polygon edge that penetrates the capsule the most
    for (int i = 0; i < n; i++) {
        const Vec2& edgeA = verts[i];
        const Vec2& edgeB = verts[(i + 1) % n];

        Vec2 closestOnEdge, closestOnCapsule;
        ClosestPointsOnTwoLineSegments(edgeA, edgeB, capA, capB, closestOnEdge, closestOnCapsule);

        const Vec2 diff = closestOnCapsule - closestOnEdge;
        const float distSq = diff.MagnitudeSquared();

        // If the squared distance is greater than the squared radius, there is no collision with this edge
        if (distSq >= radius * radius) {
            continue;
        }

        const float dist = std::sqrt(distSq);
        const float depth = radius - dist;

        // Calculate the edge normal 
        Vec2 edgeVec = edgeB - edgeA;
        Vec2 edgeNorm = Vec2(-edgeVec.y, edgeVec.x).UnitVector();

        // If the distance is close to zero (exact corner), we use the edge normal as a fallback
        Vec2 normal = (dist > mage::math::EPSILON) ? (diff / dist) : edgeNorm;

        // Ensure the normal always points in the correct direction (from capsule to polygon)
        if (normal.Dot(edgeNorm) < 0.0f) {
            normal = -normal;
        }

        // Store the deepest collision found so far
        if (depth > maxDepth) {
            maxDepth = depth;
            bestNormal = normal;
            bestOnEdge = closestOnEdge;
            bestOnCapsule = closestOnCapsule;
            bestEdgeA = edgeA;
            bestEdgeB = edgeB;
            found = true;
        }
    }

    if (!found) {
        return false;
    }

    // Unicycle Protection (Try to generate 2 contact points if lying flat)
    Vec2 spine = capB - capA;
    Vec2 spineDir = spine.MagnitudeSquared() > mage::math::EPSILON_SQ ? spine.UnitVector() : Vec2(0.0f, 1.0f);

    // Check if the spine is lying flat relative to the collision normal
    if (std::abs(spineDir.Dot(bestNormal)) < 0.15f) {
        bool addedTwoPoints = false;
        Vec2 points[2] = { capA, capB };

        for (int i = 0; i < 2; i++) {
            Vec2 p = points[i];
            Vec2 closestOnEdge = ClosestPointOnLineSegment(p, bestEdgeA, bestEdgeB);
            Vec2 diff = p - closestOnEdge;
            float dist = diff.Magnitude();

            // If the tip is still physically directly above the polygon edge
            if (dist < radius + mage::math::EPSILON) {
                float depth = radius - dist;
                Contact c;
                c.a = capsule;
                c.b = poly;
                c.normal = bestNormal;
                c.depth = depth;
                c.start = p - bestNormal * radius; // On the capsule's skin/surface
                c.end = c.start + bestNormal * depth;
                contacts.emplace_back(c);
                addedTwoPoints = true;
            }
        }

        if (addedTwoPoints) {
            return true;
        }
    }

    // Fallback: Simple 1-point collision (tip, sharp corner, or slipping off)
    const Vec2 contactPoint = bestOnCapsule - bestNormal * radius;
    Contact contact;
    contact.a = capsule;
    contact.b = poly;
    contact.start = contactPoint;
    contact.end = contactPoint + bestNormal * maxDepth;
    contact.normal = bestNormal;
    contact.depth = maxDepth;
    contacts.emplace_back(contact);

    return true;
}

bool CollisionDetection::IsCollidingPolygonPolygon(Body* a, Body* b, std::vector<Contact>& contacts) {
    PolygonShape* aPolygonShape = static_cast<PolygonShape*>(a->shape.get());
    PolygonShape* bPolygonShape = static_cast<PolygonShape*>(b->shape.get());

    int aIndexReferenceEdge, bIndexReferenceEdge;
    Vec2 aSupportPoint, bSupportPoint;

    float abSeparation = aPolygonShape->FindMinSeparation(bPolygonShape, aIndexReferenceEdge, aSupportPoint);
    if (abSeparation >= 0) return false;
    
    float baSeparation = bPolygonShape->FindMinSeparation(aPolygonShape, bIndexReferenceEdge, bSupportPoint);
    if (baSeparation >= 0) return false;

    PolygonShape* referenceShape;
    PolygonShape* incidentShape;
    int indexReferenceEdge;

    if (abSeparation > baSeparation) {
        referenceShape = aPolygonShape;
        incidentShape = bPolygonShape;
        indexReferenceEdge = aIndexReferenceEdge;
    }
    else {
        referenceShape = bPolygonShape;
        incidentShape = aPolygonShape;
        indexReferenceEdge = bIndexReferenceEdge;
    }

    // Find the reference edge based on the index that returned from the function
    Vec2 referenceEdge = referenceShape->EdgeAt(indexReferenceEdge);

    // Clipping (Sutherland-Hodgman)
    int incidentIndex = incidentShape->FindIncidentEdge(referenceEdge.Normal());
    int incidentNextIndex = (incidentIndex + 1) % incidentShape->worldVertices.size();
    Vec2 v0 = incidentShape->worldVertices[incidentIndex];
    Vec2 v1 = incidentShape->worldVertices[incidentNextIndex];

    std::vector<Vec2> contactPoints = { v0, v1 };
    std::vector<Vec2> clippedPoints;
    contactPoints.reserve(2);
    clippedPoints.reserve(2);
    clippedPoints = contactPoints;

    for (size_t i = 0; i < referenceShape->worldVertices.size(); i++) {
        if (i == indexReferenceEdge)
            continue;

        Vec2 c0 = referenceShape->worldVertices[i];
        Vec2 c1 = referenceShape->worldVertices[(i + 1) % referenceShape->worldVertices.size()];

        int numClipped = referenceShape->ClipSegmentToLine(contactPoints, clippedPoints, c0, c1);
        if (numClipped < 2) {
            break;
        }

        contactPoints = clippedPoints; // make the next contact points the ones that were just clipped
    }

    const Vec2& vRef = referenceShape->worldVertices[indexReferenceEdge];

    // Loop all clipped points, but only consider those where separation is negative (objects are penetrating)
    for (auto& vclip : clippedPoints) {
        float separation = (vclip - vRef).Dot(referenceEdge.Normal());
        if (separation <= 0) {
            Contact contact;
            contact.a = a;
            contact.b = b;
            contact.normal = referenceEdge.Normal();
            contact.start = vclip;
            contact.end = vclip + contact.normal * -separation;

            if (baSeparation >= abSeparation) {
                std::swap(contact.start, contact.end); // the start-end points are always from "a" to "b"
                contact.normal *= -1.0f;               // the collision normal is always from "a" to "b"
            }

            contacts.push_back(contact);
        }
    }
    return true;
}

bool CollisionDetection::IsCollidingPolygonCircle(Body* polygon, Body* circle, std::vector<Contact>& contacts) {
    const PolygonShape* polygonShape = static_cast<PolygonShape*>(polygon->shape.get());
    const CircleShape* circleShape = static_cast<CircleShape*>(circle->shape.get());
    const std::vector<Vec2>& polygonVertices = polygonShape->worldVertices;

    bool isOutside = false;
    Vec2 minCurrVertex;
    Vec2 minNextVertex;
    float distanceCircleEdge = std::numeric_limits<float>::lowest();

    // Loop all the edges of the polygon/box finding the nearest edge to the circle center
    for (size_t i = 0; i < polygonVertices.size(); i++) {
        int currVertex = i;
        int nextVertex = (i + 1) % polygonVertices.size();
        Vec2 edge = polygonShape->EdgeAt(currVertex);
        Vec2 normal = edge.Normal();

        // Compare the circle center with the rectangle vertex
        Vec2 vertexToCircleCenter = circle->position - polygonVertices[currVertex];
        float projection = vertexToCircleCenter.Dot(normal);

        // If we found a dot product projection that is in the positive/outside side of the normal
        if (projection > 0) {
            // Circle center is outside the polygon
            distanceCircleEdge = projection;
            minCurrVertex = polygonVertices[currVertex];
            minNextVertex = polygonVertices[nextVertex];
            isOutside = true;
            break;
        }
        else {
            // Circle center is inside the rectangle, find the min edge (the one with the least negative projection)
            if (projection > distanceCircleEdge) {
                distanceCircleEdge = projection;
                minCurrVertex = polygonVertices[currVertex];
                minNextVertex = polygonVertices[nextVertex];
            }
        }
    }

    Contact contact;

    if (isOutside) {
        // Check if we are inside region A (Vertex 1)
        Vec2 v1 = circle->position - minCurrVertex; // vector from the nearest vertex to the circle center
        Vec2 v2 = minNextVertex - minCurrVertex; // the nearest edge (from curr vertex to next vertex)

        if (v1.Dot(v2) < 0) {
            // Distance from vertex to circle center is greater than radius... no collision
            if (v1.Magnitude() > circleShape->radius) {
                return false;
            }
            else {
                // Detected collision in region A:
                contact.a = polygon;
                contact.b = circle;
                contact.depth = circleShape->radius - v1.Magnitude();
                contact.normal = v1.Normalize();
                contact.start = circle->position + (contact.normal * -circleShape->radius);
                contact.end = contact.start + (contact.normal * contact.depth);
            }
        }
        else {
            // Check if we are inside region B (Vertex 2)
            v1 = circle->position - minNextVertex; // vector from the next nearest vertex to the circle center
            v2 = minCurrVertex - minNextVertex;   // the nearest edge

            if (v1.Dot(v2) < 0) {
                // Distance from vertex to circle center is greater than radius... no collision
                if (v1.Magnitude() > circleShape->radius) {
                    return false;
                }
                else {
                    // Detected collision in region B:
                    contact.a = polygon;
                    contact.b = circle;
                    contact.depth = circleShape->radius - v1.Magnitude();
                    contact.normal = v1.Normalize();
                    contact.start = circle->position + (contact.normal * -circleShape->radius);
                    contact.end = contact.start + (contact.normal * contact.depth);
                }
            }
            else {
                // We are inside region C (Edge Center)
                if (distanceCircleEdge > circleShape->radius) {
                    // No collision... Distance between the closest distance and the circle center is greater than the radius.
                    return false;
                }
                else {
                    // Detected collision in region C:
                    contact.a = polygon;
                    contact.b = circle;
                    contact.depth = circleShape->radius - distanceCircleEdge;
                    contact.normal = (minNextVertex - minCurrVertex).Normal();
                    contact.start = circle->position - (contact.normal * circleShape->radius);
                    contact.end = contact.start + (contact.normal * contact.depth);
                }
            }
        }
    }
    else {
        // The center of circle is inside the polygon
        contact.a = polygon;
        contact.b = circle;
        contact.depth = circleShape->radius - distanceCircleEdge;
        contact.normal = (minNextVertex - minCurrVertex).Normal();
        contact.start = circle->position - (contact.normal * circleShape->radius);
        contact.end = contact.start + (contact.normal * contact.depth);
    }

    contacts.push_back(contact);

    return true;
}

bool CollisionDetection::RaycastCircle(Body* circleBody, const Vec2& start, const Vec2& end, RaycastResult& out) {
    CircleShape* circle = static_cast<CircleShape*>(circleBody->shape.get());
    Vec2  center = circleBody->position;
    float radius = circle->radius;

    Vec2 rayVector = end - start;

    Vec2 offset = start - center;

    float quadA = rayVector.Dot(rayVector);
    float quadB = 2.0f * offset.Dot(rayVector);
    float quadC = offset.Dot(offset) - (radius * radius);

    float discriminant = (quadB * quadB) - (4.0f * quadA * quadC);

    if (discriminant < 0.0f) return false;

    float sqrtDisc = std::sqrt(discriminant);

    float t = (-quadB - sqrtDisc) / (2.0f * quadA);

    if (t < 0.0f)
        t = (-quadB + sqrtDisc) / (2.0f * quadA);

    if (t < 0.0f || t > 1.0f) return false;

    out.hit = true;
    out.body = circleBody;
    out.t = t;
    out.point = start + rayVector * t;
    out.normal = (out.point - center).UnitVector();

    return true;
}

bool CollisionDetection::RaycastPolygon(Body* polygonBody, const Vec2& start, const Vec2& end, RaycastResult& out) {
    PolygonShape* poly = static_cast<PolygonShape*>(polygonBody->shape.get());
    const auto& vertices = poly->worldVertices;
    int numVertices = (int)vertices.size();

    if (numVertices == 0) return false;

    Vec2  rayDir = end - start;
    float closestT = 1.0f;
    bool  hit = false;

    for (int i = 0; i < numVertices; i++) {
        Vec2 v1 = vertices[i];
        Vec2 v2 = vertices[(i + 1) % numVertices];
        Vec2 edgeDir = v2 - v1;

        float cross = rayDir.Cross(edgeDir);

        // Ray and edge are parallel
        if (std::abs(cross) < mage::math::EPSILON) continue;

        Vec2  delta = v1 - start;
        float t = delta.Cross(edgeDir) / cross;
        float u = delta.Cross(rayDir) / cross;

        // Check if intersection occurs within the line segment bounds
        if (t >= 0.0f && t <= closestT && u >= 0.0f && u <= 1.0f) {
            closestT = t;
            hit = true;

            out.hit = true;
            out.body = polygonBody;
            out.t = closestT;
            out.point = start + rayDir * closestT;

            // Calculate perpendicular normal to the edge
            Vec2 normal = Vec2(-edgeDir.y, edgeDir.x).UnitVector();

            // Ensure the normal points AGAINST the ray (towards the origin)
            // If the ray and normal point in the same direction, invert the normal
            if (normal.Dot(rayDir) > 0.0f) {
                normal = -normal;
            }

            out.normal = normal;
        }
    }

    return hit;
}

bool CollisionDetection::RaycastCapsule(Body* capsuleBody, const Vec2& start, const Vec2& end, RaycastResult& out) {
    const CapsuleShape* capShape = static_cast<const CapsuleShape*>(capsuleBody->shape.get());
    const float radius = capShape->radius;
    const Vec2& capA = capShape->worldP1;
    const Vec2& capB = capShape->worldP2;

    Vec2 rayDir = end - start;
    float minT = 1.0f; // Limit hits to the ray length (t in [0, 1])
    bool hit = false;
    Vec2 bestNormal(0.0f, 0.0f);

    // Helper lambda to test Ray vs Line Segment (for the straight sides of the capsule)
    auto checkSegment = [&](const Vec2& p1, const Vec2& p2) {
        Vec2 edgeDir = p2 - p1;
        float cross = rayDir.Cross(edgeDir);

        // Ray and edge are parallel
        if (std::abs(cross) < mage::math::EPSILON) return;

        Vec2 delta = p1 - start;
        float t = delta.Cross(edgeDir) / cross;
        float u = delta.Cross(rayDir) / cross;

        // Check if intersection occurs within the segment bounds and is the closest one so far
        if (t >= 0.0f && t < minT && u >= 0.0f && u <= 1.0f) {
            minT = t;
            hit = true;

            // Calculate outward normal
            Vec2 n = Vec2(-edgeDir.y, edgeDir.x).UnitVector();
            if (n.Dot(rayDir) > 0.0f) {
                n = -n;
            }
            bestNormal = n;
        }
        };

    // Helper lambda to test Ray vs Circle (for the round caps)
    auto checkCircle = [&](const Vec2& center) {
        Vec2 offset = start - center;
        float a = rayDir.Dot(rayDir);
        float b = 2.0f * offset.Dot(rayDir);
        float c = offset.Dot(offset) - (radius * radius);

        float discriminant = (b * b) - (4.0f * a * c);

        // No real roots, no intersection
        if (discriminant < 0.0f) return;

        float sqrtDisc = std::sqrt(discriminant);

        // First intersection point (closest)
        float t1 = (-b - sqrtDisc) / (2.0f * a);

        if (t1 >= 0.0f && t1 < minT) {
            minT = t1;
            hit = true;
            Vec2 hitPoint = start + rayDir * t1;
            bestNormal = (hitPoint - center).UnitVector();
        }
        else {
            // Second intersection point (if the ray starts inside the circle)
            float t2 = (-b + sqrtDisc) / (2.0f * a);
            if (t2 >= 0.0f && t2 < minT) {
                minT = t2;
                hit = true;
                Vec2 hitPoint = start + rayDir * t2;
                bestNormal = (hitPoint - center).UnitVector();
            }
        }
        };

    // Test against the two round caps (Circles)
    checkCircle(capA);
    checkCircle(capB);

    // Test against the two straight side walls (Segments)
    Vec2 spine = capB - capA;
    if (spine.MagnitudeSquared() > mage::math::EPSILON_SQ) {
        Vec2 spineDir = spine.UnitVector();

        // Normal vector perpendicular to the spine, scaled by the radius
        Vec2 normalOffset = Vec2(-spineDir.y, spineDir.x) * radius;

        // Side 1
        checkSegment(capA + normalOffset, capB + normalOffset);
        // Side 2
        checkSegment(capA - normalOffset, capB - normalOffset);
    }

    // Compile the final result if any hit was detected
    if (hit) {
        out.hit = true;
        out.body = capsuleBody;
        out.t = minT;
        out.point = start + rayDir * minT;
        out.normal = bestNormal;
        return true;
    }

    return false;
}
