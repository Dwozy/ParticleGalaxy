/*
 * Interface file for the coarse collision detection system.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

/**
 * @file
 *
 * This file contains the coarse collision detection system.  It is
 * used to return pairs of objects that may be in contact, which can
 * then be tested using fined grained methods.
 */



#ifndef CYCLONE_COLLISION_COARSE_H
#define CYCLONE_COLLISION_COARSE_H

#include <vector>
#include <cstddef>
#include "contacts.h"
#include "particle.h"



static int index = 1;

namespace cyclone {

    
    class BoundingBox
    {
    public:

        cyclone::Vector3 corner1;
        cyclone::Vector3 corner2;

        BoundingBox() {

            corner1 = cyclone::Vector3(0, 0, 0);
            corner2 = cyclone::Vector3(0, 0, 0);
        };

        BoundingBox(BoundingBox* v)
        {
            corner1 = v->corner1;
            corner2 = v->corner2;
        }

        BoundingBox(cyclone::Vector3 c1, cyclone::Vector3 c2)
        {
            
            this->corner1 = c1;
            this->corner2 = c2;

            cyclone::Vector3 minA, maxA;
            this->getMinMax(minA, maxA);
            corner1 = minA;
            corner2 = maxA;
        }

        BoundingBox(BoundingBox& v)
        {

            corner1 = v.corner1;
            corner2 = v.corner2;
            cyclone::Vector3 minA, maxA;
            this->getMinMax(minA, maxA);
            corner1 = minA;
            corner2 = maxA;
        }

        void getMinMax(cyclone::Vector3& min, cyclone::Vector3& max) const
        {
            min = { corner1.x, corner1.y, corner1.z };
            max = { corner2.x, corner2.y, corner2.z };

            if (corner1.x < min.x) min.x = corner1.x;
            if (corner1.y < min.y) min.y = corner1.y;
            if (corner1.z < min.z) min.z = corner1.z;
            if (corner1.x > max.x) max.x = corner1.x;
            if (corner1.y > max.y) max.y = corner1.y;
            if (corner1.z > max.z) max.z = corner1.z;

            if (corner2.x < min.x) min.x = corner2.x;
            if (corner2.y < min.y) min.y = corner2.y;
            if (corner2.z < min.z) min.z = corner2.z;
            if (corner2.x > max.x) max.x = corner2.x;
            if (corner2.y > max.y) max.y = corner2.y;
            if (corner2.z > max.z) max.z = corner2.z;

        
        }

        bool isInside(const BoundingBox& other) const {
            
            cyclone::Vector3 minA, maxA, minB, maxB;
            this->getMinMax(minA, maxA);
            other.getMinMax(minB, maxB);

            return (minB.x >= minA.x && maxB.x <= maxA.x) &&
                (minB.y >= minA.y && maxB.y <= maxA.y) &&
                (minB.z >= minA.z && maxB.z <= maxA.z);
        }



        bool operator==(BoundingBox& v)
        {
            if (((v.corner1 - corner1).magnitude() < 0.01f) && ((v.corner2 - corner2).magnitude() < 0.01f))
                return true;

            return false;

        }

        BoundingBox(const BoundingBox& one, const BoundingBox& two)
        {
           

            // Check if the larger sphere encloses the small one
            if (one.isInside(two))
            {

                corner1 = two.corner1;
                corner2 = two.corner2;
            }
            else if (two.isInside(one))
            {
                corner1 = one.corner1;
                corner2 = one.corner2;
            }

            else {

                cyclone::Vector3 min1, max1, min2, max2;
                one.getMinMax(min1, max1);
                two.getMinMax(min2, max2);

                cyclone::Vector3 boundingMin = {
                  std::min(min1.x, min2.x),
                  std::min(min1.y, min2.y),
                  std::min(min1.z, min2.z)
                };

                cyclone::Vector3 boundingMax = {
                    std::max(max1.x, max2.x),
                    std::max(max1.y, max2.y),
                    std::max(max1.z, max2.z)
                };

                corner1 = boundingMin;
                corner2 = boundingMax;

            }           

        }
 
        bool overlaps(const BoundingBox& other) const
        {

            cyclone::Vector3 minA, maxA, minB, maxB;
            this->getMinMax(minA, maxA);
            other.getMinMax(minB, maxB);


            /*
            if (corner1.x < other.corner2.x || corner2.x > other.corner1.x) return false;
            if (corner1.y < other.corner2.y || corner2.y > other.corner1.y) return false;
            if (corner1.z < other.corner2.z || corner2.z > other.corner1.z) return false;
            */
                        
            if (maxA[0] < minB[0] || minA[0] > maxB[0]) {
                return false;
            }
            
            // Check overlap on the y-axis
            if (maxA[1] < minB[1] || minA[1] > maxB[1]) {
                return false;
            }
            
            // Check overlap on the z-axis
            if (maxA[2] < minB[2] || minA[2] > maxB[2]) {
                return false;
            }
            
            

            return true;
        }

        real getGrowth(BoundingBox& other) const
        {
           BoundingBox newSphere(*this, other);
                                
            // Calculate the lengths of the edges of the box
           float length = std::abs(corner1.x - corner2.x);
           float width = std::abs(corner1.y - corner2.y);
           float height = std::abs(corner1.z - corner2.z);

                // Calculate the volume
           float volume = length * width * height;
          
           float length2 = std::abs(newSphere.corner1.x - newSphere.corner2.x);
           float width2 = std::abs(newSphere.corner1.y - newSphere.corner2.y);
           float height2 = std::abs(newSphere.corner1.z - newSphere.corner2.z);

           float volume2 = length2 * width2 * height2;
            //  std::cout << r << " grow" << std::endl;
            return volume2-volume;

        }

        real getSize() const
        {
            float length = std::abs(corner1.x - corner2.x);
            float width = std::abs(corner1.y - corner2.y);
            float height = std::abs(corner1.z - corner2.z);

            // Calculate the volume
            float volume = length * width * height;
            return volume;

        }
    };


    class BoundingSphere
    {
    public:
        cyclone::Vector3 centre;
        real radius;

      

        /**
         * Creates a new bounding sphere at the given centre and radius.
         */

        BoundingSphere() {
            centre = cyclone::Vector3(0, 0, 0);
            radius = 0;
        };

        BoundingSphere(cyclone::Vector3 &centre, real radius);

        BoundingSphere(BoundingSphere * v);

        BoundingSphere(BoundingSphere & v)
        {
            centre = v.centre;
            radius = v.radius;
      
          
        }

        bool operator==(BoundingSphere& v)
        {
            if ((v.centre - centre).magnitude() < 0.001 && ((v.radius - radius) < 0.001))
                return true;
            return false;

        }

        /**
         * Creates a bounding sphere to enclose the two given bounding
         * spheres.
         */
 
        BoundingSphere(const BoundingSphere & one, const BoundingSphere & two);
        ~BoundingSphere() {};

        /**
         * Checks if the bounding sphere overlaps with the other given
         * bounding sphere.
         */
        bool overlaps( BoundingSphere * other) const;
        bool overlaps(const BoundingSphere & other) const;
        /**
         * Reports how much this bounding sphere would have to grow
         * by to incorporate the given bounding sphere. Note that this
         * calculation returns a value not in any particular units (i.e.
         * its not a volume growth). In fact the best implementation
         * takes into account the growth in surface area (after the
         * Goldsmith-Salmon algorithm for tree construction).
         */
        real getGrowth( BoundingSphere & other) const;

        /**
         * Returns the volume of this bounding volume. This is used
         * to calculate how to recurse into the bounding volume tree.
         * For a bounding sphere it is a simple calculation.
         */
        real getSize() const
        {
            return ((real)1.333333) * R_PI * radius * radius * radius;
        }
    };

    /**
     * Stores a potential contact to check later.
     */
    struct PotentialContact
    {
        /**
         * Holds the bodies that might be in contact.
         */
        RigidBody* body[2];
  
    };
    

    struct PotentialContact2
    {
        /**
         * Holds the bodies that might be in contact.
         */
        cyclone::Particle* body[2];

    };




    /**
     * A base class for nodes in a bounding volume hierarchy.
     *
     * This class uses a binary tree to store the bounding
     * volumes.
     */



    template<class BoundingVolumeClass>
    class BVHNode
    {
    public:

        int id;
        /**
         * Holds the child nodes of this node.
         */
        BVHNode * children[2];

        /**
         * Holds a single bounding volume encompassing all the
         * descendents of this node.
         */
        BoundingVolumeClass volume;

        /**
         * Holds the rigid body at this node of the hierarchy.
         * Only leaf nodes can have a rigid body defined (see isLeaf).
         * Note that it is possible to rewrite the algorithms in this
         * class to handle objects at all levels of the hierarchy,
         * but the code provided ignores this vector unless firstChild
         * is NULL.
         */
        RigidBody * body;

        // ... other BVHNode code as before ...

        /**
         * Holds the node immediately above us in the tree.
         */
        BVHNode * parent;

        /**
         * Creates a new node in the hierarchy with the given parameters.
         */
        BVHNode()
        {

        }


        BVHNode(BVHNode* parent, BoundingVolumeClass & volume, RigidBody* body = NULL)
            : parent(parent), volume(volume), body(body)
        {
           // id = index++;
            id = 0;
        //    std::cout << id << " created" << std::endl;
            children[0] = children[1] = NULL;
        }

        /**
         * Checks if this node is at the bottom of the hierarchy.
         */
        bool isLeaf() const
        {
            return (body != NULL);
        }

    
        /**
         * Checks the potential contacts from this node downwards in
         * the hierarchy, writing them to the given array (up to the
         * given limit). Returns the number of potential contacts it
         * found.
         */
        unsigned getPotentialContacts(PotentialContact* contacts, unsigned limit) ;
  
        unsigned getPotentialContacts2(const BVHNode<BoundingVolumeClass >* other, PotentialContact* contacts, unsigned limit);

        /**
         * Inserts the given rigid body, with the given bounding volume,
         * into the hierarchy. This may involve the creation of
         * further bounding volume nodes.
         */
        void insert(RigidBody* body, BoundingVolumeClass & volume);

        /**
         * Deltes this node, removing it first from the hierarchy, along
         * with its associated
         * rigid body and child nodes. This method deletes the node
         * and all its children (but obviously not the rigid bodies). This
         * also has the effect of deleting the sibling of this node, and
         * changing the parent node so that it contains the data currently
         * in that sibling. Finally it forces the hierarchy above the
         * current node to reconsider its bounding volume.
         */
        ~BVHNode();

    protected:

        /**
         * Checks for overlapping between nodes in the hierarchy. Note
         * that any bounding volume should have an overlaps method implemented
         * that checks for overlapping with another object of its own type.
         */
        bool overlaps(const BVHNode<BoundingVolumeClass > * other);

        /**
         * Checks the potential contacts between this node and the given
         * other node, writing them to the given array (up to the
         * given limit). Returns the number of potential contacts it
         * found.
         */
        unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass > *other, PotentialContact* contacts, unsigned limit) ;

        /**
         * For non-leaf nodes, this method recalculates the bounding volume
         * based on the bounding volumes of its children.
         */
        void recalculateBoundingVolume(bool recurse = true);
    };


    //below class for particle not for rigid body


    template<class BoundingVolumeClass>
    class BVHNode2
    {
    public:

        int id;
        /**
         * Holds the child nodes of this node.
         */
        BVHNode2* children[2];

        /**
         * Holds a single bounding volume encompassing all the
         * descendents of this node.
         */
        BoundingVolumeClass volume;

        /**
         * Holds the rigid body at this node of the hierarchy.
         * Only leaf nodes can have a rigid body defined (see isLeaf).
         * Note that it is possible to rewrite the algorithms in this
         * class to handle objects at all levels of the hierarchy,
         * but the code provided ignores this vector unless firstChild
         * is NULL.
         */
        Particle* body;

        // ... other BVHNode code as before ...

        /**
         * Holds the node immediately above us in the tree.
         */
        BVHNode2* parent;

        /**
         * Creates a new node in the hierarchy with the given parameters.
         */
        BVHNode2()
        {

        }


        BVHNode2(BVHNode2* parent, BoundingVolumeClass& volume, Particle* body = NULL)
            : parent(parent), volume(volume), body(body)
        {
            // id = index++;
            id = 0;
            //    std::cout << id << " created" << std::endl;
            children[0] = children[1] = NULL;
        }

        /**
         * Checks if this node is at the bottom of the hierarchy.
         */
        bool isLeaf() const
        {
            return (body != NULL);
        }


        /**
         * Checks the potential contacts from this node downwards in
         * the hierarchy, writing them to the given array (up to the
         * given limit). Returns the number of potential contacts it
         * found.
         */
        unsigned getPotentialContacts(PotentialContact2* contacts, unsigned limit);

        unsigned getPotentialContacts2(const BVHNode2<BoundingVolumeClass >* other, PotentialContact2* contacts, unsigned limit);

        /**
         * Inserts the given rigid body, with the given bounding volume,
         * into the hierarchy. This may involve the creation of
         * further bounding volume nodes.
         */
        void insert(Particle* body, BoundingVolumeClass& volume);

        /**
         * Deltes this node, removing it first from the hierarchy, along
         * with its associated
         * rigid body and child nodes. This method deletes the node
         * and all its children (but obviously not the rigid bodies). This
         * also has the effect of deleting the sibling of this node, and
         * changing the parent node so that it contains the data currently
         * in that sibling. Finally it forces the hierarchy above the
         * current node to reconsider its bounding volume.
         */
        ~BVHNode2();

    protected:

        /**
         * Checks for overlapping between nodes in the hierarchy. Note
         * that any bounding volume should have an overlaps method implemented
         * that checks for overlapping with another object of its own type.
         */
        bool overlaps(const BVHNode2<BoundingVolumeClass >* other);

        /**
         * Checks the potential contacts between this node and the given
         * other node, writing them to the given array (up to the
         * given limit). Returns the number of potential contacts it
         * found.
         */
        unsigned getPotentialContactsWith(const BVHNode2<BoundingVolumeClass >* other, PotentialContact2* contacts, unsigned limit);

        /**
         * For non-leaf nodes, this method recalculates the bounding volume
         * based on the bounding volumes of its children.
         */
        void recalculateBoundingVolume(bool recurse = true);
    };




    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    template<class BoundingVolumeClass> bool BVHNode<BoundingVolumeClass>::overlaps(const BVHNode<BoundingVolumeClass > * other) 
    {

   //     return volume.overlaps(other.volume);
        
          return  volume.overlaps(other->volume);
        
    }



    template<class BoundingVolumeClass> bool BVHNode2<BoundingVolumeClass>::overlaps(const BVHNode2<BoundingVolumeClass >* other)
    {

        //     return volume.overlaps(other.volume);

        return  volume.overlaps(other->volume);

    }




 
    template<class BoundingVolumeClass> void BVHNode<BoundingVolumeClass>::insert(RigidBody* newBody, BoundingVolumeClass &  newVolume)
    {
        // If we are a leaf, then the only option is to spawn two
        // new children and place the new body in one.
        if (isLeaf())
        {
           

            // Child one is a copy of us.
            children[0] = new BVHNode<BoundingVolumeClass>(
                this, volume, body
            );
          

            // Child two holds the new body
            children[1] = new BVHNode<BoundingVolumeClass>(
                this, newVolume, newBody
            );
          
            // And we now loose the body (we're no longer a leaf)
            this->body = NULL;

            // We need to recalculate our bounding volume
            recalculateBoundingVolume();


        }

        // Otherwise we need to work out which child gets to keep
        // the inserted body. We give it to whoever would grow the
        // least to incorporate it.
        else
        {
            real a = children[0]->volume.getGrowth(newVolume);
            real b = children[1]->volume.getGrowth(newVolume);

           
            if (a  < b)
            {
                children[0]->insert(newBody, newVolume);
            }
            else
            {
                children[1]->insert(newBody, newVolume);
            }
        }
    }



    template<class BoundingVolumeClass> void BVHNode2<BoundingVolumeClass>::insert(Particle* newBody, BoundingVolumeClass& newVolume)
    {
        // If we are a leaf, then the only option is to spawn two
        // new children and place the new body in one.
        if (isLeaf())
        {


            // Child one is a copy of us.
            children[0] = new BVHNode2<BoundingVolumeClass>(
                this, volume, body
            );


            // Child two holds the new body
            children[1] = new BVHNode2<BoundingVolumeClass>(
                this, newVolume, newBody
            );

            // And we now loose the body (we're no longer a leaf)
            this->body = NULL;

            // We need to recalculate our bounding volume
            recalculateBoundingVolume();


        }

        // Otherwise we need to work out which child gets to keep
        // the inserted body. We give it to whoever would grow the
        // least to incorporate it.
        else
        {
            real a = children[0]->volume.getGrowth(newVolume);
            real b = children[1]->volume.getGrowth(newVolume);


            if (a < b)
            {
                children[0]->insert(newBody, newVolume);
            }
            else
            {
                children[1]->insert(newBody, newVolume);
            }
        }
    }


    template<class BoundingVolumeClass> BVHNode<BoundingVolumeClass>::~BVHNode()
    {

      //  std::cout << id << " deleted" << std::endl;
           
        /*
        if (parent) {
            BVHNode<BoundingVolumeClass>** siblings = parent->children;
            if (siblings[0] == this) {
                siblings[0] = NULL;
            }
            else {
                siblings[1] = NULL;
            }
            parent->recalculateBoundingVolume();
        }
        if (children[0]) delete children[0];
        if (children[1]) delete children[1];
        */

        
        // If we don't have a parent, then we ignore the sibling
        // processing
        if (parent)
        {
            // Find our sibling
            BVHNode<BoundingVolumeClass>* sibling;
            if (parent->children[0] == this) sibling = parent->children[1];
            else sibling = parent->children[0];

            // Write its data to our parent
            parent->volume = sibling->volume;
            parent->body = sibling->body;
            parent->children[0] = sibling->children[0];
            parent->children[1] = sibling->children[1];

            // Delete the sibling (we blank its parent and
            // children to avoid processing/deleting them)
            sibling->parent = NULL;
            sibling->body = NULL;
            sibling->children[0] = NULL;
            sibling->children[1] = NULL;
            sibling->volume  = NULL;
            delete sibling;

            // Recalculate the parent's bounding volume
            parent->recalculateBoundingVolume();
        }

        
        // Delete our children (again we remove their
        // parent data so we don't try to process their siblings
        // as they are deleted).
        if (children[0]) {
            children[0]->parent = NULL;
            delete children[0];
        }
        if (children[1]) {
            children[1]->parent = NULL;
            delete children[1];
        }
        
        
    }


    template<class BoundingVolumeClass> BVHNode2<BoundingVolumeClass>::~BVHNode2()
    {

          // If we don't have a parent, then we ignore the sibling
          // processing
        if (parent)
        {
            // Find our sibling
            BVHNode2<BoundingVolumeClass>* sibling;
            if (parent->children[0] == this) sibling = parent->children[1];
            else sibling = parent->children[0];

            // Write its data to our parent
            parent->volume = sibling->volume;
            parent->body = sibling->body;
            parent->children[0] = sibling->children[0];
            parent->children[1] = sibling->children[1];

            // Delete the sibling (we blank its parent and
            // children to avoid processing/deleting them)
            sibling->parent = NULL;
            sibling->body = NULL;
            sibling->children[0] = NULL;
            sibling->children[1] = NULL;
            sibling->volume = NULL;
            delete sibling;

            // Recalculate the parent's bounding volume
            parent->recalculateBoundingVolume();
        }


        // Delete our children (again we remove their
        // parent data so we don't try to process their siblings
        // as they are deleted).
        if (children[0]) {
            children[0]->parent = NULL;
            delete children[0];
        }
        if (children[1]) {
            children[1]->parent = NULL;
            delete children[1];
        }


    }


    template<class BoundingVolumeClass> void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(bool recurse)
    {
        if (isLeaf()) return;

        // Use the bounding volume combining constructor.

        volume = BoundingVolumeClass(
            children[0]->volume,
            children[1]->volume
        );
        

        // Recurse up the tree
        if (parent) parent->recalculateBoundingVolume(true);

       
    }


    template<class BoundingVolumeClass> void BVHNode2<BoundingVolumeClass>::recalculateBoundingVolume(bool recurse)
    {
        if (isLeaf()) return;

        // Use the bounding volume combining constructor.

        volume = BoundingVolumeClass(
            children[0]->volume,
            children[1]->volume
        );


        // Recurse up the tree
        if (parent) parent->recalculateBoundingVolume(true);


    }




    template<class BoundingVolumeClass> unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(PotentialContact* contacts, unsigned limit) 
    {
        // Early out if we don't have the room for contacts, or
        // if we're a leaf node.
        if (isLeaf() || limit == 0) return 0;

        // Get the potential contacts of one of our children with
        // the other
      
       
        return children[0]->getPotentialContactsWith(children[1], contacts, limit);       

    }

    template<class BoundingVolumeClass> unsigned BVHNode2<BoundingVolumeClass>::getPotentialContacts(PotentialContact2* contacts, unsigned limit)
    {
        // Early out if we don't have the room for contacts, or
        // if we're a leaf node.
        if (isLeaf() || limit == 0) return 0;

        // Get the potential contacts of one of our children with
        // the other


        return children[0]->getPotentialContactsWith(children[1], contacts, limit);

    }


    template<class BoundingVolumeClass> unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts2(const BVHNode<BoundingVolumeClass >* other, PotentialContact* contacts, unsigned limit)
    {
        bool f = this->overlaps(other);
        if (f &&  this->body != other->body ) {
            if (this->isLeaf() && other->isLeaf())
            {
                if (this->body->id != other->body->id) {
                    
                    contacts->body[0] = this->body;
                    contacts->body[1] = other->body;

                 //   std::cout << this->body->id << " : " << other->body->id << "    potential collide" << std::endl;

                    return 1;
                    
                }
            }
            else {
                unsigned count;
                count = children[0]->getPotentialContacts2(other, contacts, limit);
                if (limit > count) {
                    return count + children[1]->getPotentialContacts2(other, contacts + count, limit - count);
                }
                else {
                    return count;
                }
            }
        }
        return 0;
                        
      //  return this->getPotentialContactsWith(other, contacts, limit);

    }


    template<class BoundingVolumeClass> unsigned BVHNode2<BoundingVolumeClass>::getPotentialContacts2(const BVHNode2<BoundingVolumeClass >* other, PotentialContact2* contacts, unsigned limit)
    {
        bool f = this->overlaps(other);
        if (f && this->body != other->body) {
            if (this->isLeaf() && other->isLeaf())
            {
               

                    contacts->body[0] = this->body;
                    contacts->body[1] = other->body;

                    //   std::cout << this->body->id << " : " << other->body->id << "    potential collide" << std::endl;

                    return 1;

               
            }
            else {
                unsigned count;
                count = children[0]->getPotentialContacts2(other, contacts, limit);
                if (limit > count) {
                    return count + children[1]->getPotentialContacts2(other, contacts + count, limit - count);
                }
                else {
                    return count;
                }
            }
        }
        return 0;

        //  return this->getPotentialContactsWith(other, contacts, limit);

    }

    template<class BoundingVolumeClass> unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(const BVHNode<BoundingVolumeClass > * other, PotentialContact* contacts, unsigned limit) 
    {
        // Early out if we don't overlap or if we have no room
         // to report contacts
         // 

        // If we're both at leaf nodes, then we have a potential contact

        if (isLeaf() && other->isLeaf()) //only two leafs are tested..
        {

            contacts->body[0] = body;
            contacts->body[1] = other->body;
            return 1;
        }


             
        
        // Determine which node to descend into. If either is
        // a leaf, then we descend the other. If both are branches,
        // then we use the one with the largest size.
    

        if (other->isLeaf() || (!isLeaf() && volume.getSize() >= other->volume.getSize()) )
        {
            // Recurse into ourself
            if (children[0]) {
                unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

                if (limit > count) {
                    return count + children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
                }
                else {
                    return count;
                }
            }
            
            // Check we have enough slots to do the other side too
            
        }
        else
        {
            // Recurse into the other node
            unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);

            // Check we have enough slots to do the other side too
            if (limit > count) {
                return count + getPotentialContactsWith(other->children[1], contacts + count, limit - count);
            }
            else {
                return count;
            }
        }
    }//// function end here



    template<class BoundingVolumeClass> unsigned BVHNode2<BoundingVolumeClass>::getPotentialContactsWith(const BVHNode2<BoundingVolumeClass >* other, PotentialContact2* contacts, unsigned limit)
    {
        // Early out if we don't overlap or if we have no room
         // to report contacts
         // 

        // If we're both at leaf nodes, then we have a potential contact

        if (isLeaf() && other->isLeaf()) //only two leafs are tested..
        {

            contacts->body[0] = body;
            contacts->body[1] = other->body;
            return 1;
        }




        // Determine which node to descend into. If either is
        // a leaf, then we descend the other. If both are branches,
        // then we use the one with the largest size.


        if (other->isLeaf() || (!isLeaf() && volume.getSize() >= other->volume.getSize()))
        {
            // Recurse into ourself
            if (children[0]) {
                unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

                if (limit > count) {
                    return count + children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
                }
                else {
                    return count;
                }
            }

            // Check we have enough slots to do the other side too

        }
        else
        {
            // Recurse into the other node
            unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);

            // Check we have enough slots to do the other side too
            if (limit > count) {
                return count + getPotentialContactsWith(other->children[1], contacts + count, limit - count);
            }
            else {
                return count;
            }
        }
    }//// function end here


} // namespace cyclone





#endif // CYCLONE_COLLISION_FINE_H