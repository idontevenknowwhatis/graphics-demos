Lucas Liu Project 2 Readme:
    1. Added J-spline based subdivision that converges to a B-spline.
    2. Display of control points using 'c'.
    3. Display of subdivided polyloop using 'h'.
    4. Provided keys “-” and “=” to adjust subdivision level.
    5. Provided a key “s” to display the skater, or the arrow.
    6. Subdivided polyloop is displayed using a shadow.
    7. Offset vector is computed for the foot loop.
    8. Images have been captured of the subdivision system and different levels 0-7.
    9. Walker’s speed is based on subdivision level.
    10. Construction of a basic telescopic runner with balls is done.
    11. The algorithm used to animate the horizontal/floor positions of the feet is 1/3rd based.
    12. Telescopic legs are connected between the hip and feet for each frame.
    13. Runner responds appropriately to acceleration changes, and to changes in the control polygon, leaning more into sharp turns rather than gentle ones and leaning back and forward as well.
    14. Short video of the above behavior is provided.
    15. Display of the phase 2 telescopic runner is enabled by “e”.
    16. Arms of the improved phase 3 walker are animated and have bending elbows, due to the hands following an elliptical motion.
    17. Arms are animated in such a way that they move in opposite phase to the feet.
    18. Scaling bent knees is implemented and can be enabled/disabled using the key ‘k’.
    19. Inverse kinematics is implemented and can be enabled/disabled using the key ‘k’
    20. Short video showcasing the moving arms and the inverse kinematics comparison is provided.

Extensions:
    1. Improved walker raises his feet using an algorithm based on the sine function.
    2. An alternative configuration of the walker made to resemble a dinosaur.
    3. The dinosaur’s tail is physically animated.

Issues:
    1. In certain situations when the distance between the hip at G[f] and foot at B[f] exceeds that of the maximum length possible for the leg, the improved walker will temporarily increase the lengths of its leg to accomodate this new distance.
    2. For whatever reason, the movie skips frames in a way that the walking animation for the improved walker is not well captured.