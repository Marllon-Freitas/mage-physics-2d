#include "Manifold.h"
#include "../Core/Body.h"

void Manifold::Update(const std::vector<Contact>& newContacts) {
    // Temporary arrays to hold ONLY the valid contacts for the current frame
    PenetrationConstraint newConstraints[mage::physics::MAX_CONTACTS];
    Contact newContactList[mage::physics::MAX_CONTACTS];
    int newNumContacts = 0;

    // size_t used to prevent signed/unsigned compiler mismatch warnings
    for (size_t i = 0; i < newContacts.size() && i < mage::physics::MAX_CONTACTS; i++) {
        Contact newContact = newContacts[i];
        PenetrationConstraint newConstraint(newContact.a, newContact.b, newContact.start, newContact.end, newContact.normal);

        // Feature Matching: Attempt to find a match in the OLD state to inherit Warm Start data
        for (int j = 0; j < this->numContacts; j++) {
            Vec2 aLocalDiff = newConstraint.aPoint - this->constraints[j].aPoint;
            Vec2 bLocalDiff = newConstraint.bPoint - this->constraints[j].bPoint;

            // If the local contact points are close enough, we consider it the same physical contact
            if (aLocalDiff.MagnitudeSquared() < (mage::physics::CONTACT_PROXIMITY_THRESHOLD * mage::physics::CONTACT_PROXIMITY_THRESHOLD) &&
                bLocalDiff.MagnitudeSquared() < (mage::physics::CONTACT_PROXIMITY_THRESHOLD * mage::physics::CONTACT_PROXIMITY_THRESHOLD)) {

                newConstraint.m_normalImpulse = this->constraints[j].m_normalImpulse;
                newConstraint.m_tangentImpulse = this->constraints[j].m_tangentImpulse;
                break;
            }
        }

        // 3. Add to the new confirmed list
        newContactList[newNumContacts] = newContact;
        newConstraints[newNumContacts] = newConstraint;
        newNumContacts++;
    }

    // 4. Full Cleanup: Replace old state with new state. 
    // This purges "ghost contacts" (contacts that existed last frame but not this one)
    this->numContacts = newNumContacts;
    for (int i = 0; i < newNumContacts; i++) {
        this->contacts[i] = newContactList[i];
        this->constraints[i] = newConstraints[i];
    }
}

void Manifold::PreSolve(float dt) {
    for (int i = 0; i < numContacts; i++) {
        constraints[i].PreSolve(dt);
    }
}

void Manifold::Solve() {
    for (int i = 0; i < numContacts; i++) {
        constraints[i].Solve();
    }
}

void Manifold::PostSolve() {
    for (int i = 0; i < numContacts; i++) {
        constraints[i].PostSolve();
    }
}