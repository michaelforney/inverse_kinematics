module InverseKinematics
    ( Joint (RevoluteJoint, PrismaticJoint), Link (Link), linkJoint, linkBody, linkTarget
    , linkTransform, inverseLinkTransform, jointTransform, bodyTransform, improve, dot ) where

import Graphics.Rendering.Cairo.Matrix
import Data.List
import Data.Function
import Data.Tree

data Joint = RevoluteJoint Double Double Double |
             PrismaticJoint Double Double Double
                deriving (Show, Read, Eq)

type Body = Double
data Link = Link { linkJoint :: Joint
                 , linkBody :: Body
                 , linkTarget :: Maybe (Double, Double) } deriving (Show, Read, Eq)


-- Miscellaneous Utilities {{{
dot :: (Double, Double) -> (Double, Double) -> Double
dot (x1, y1) (x2, y2) = x1 * x2 + y1 * y2

jointTransform :: Joint -> Matrix
jointTransform (RevoluteJoint _ _ rotation) = rotate rotation identity
jointTransform (PrismaticJoint _ _ position) = translate position 0 identity

bodyTransform :: Double -> Matrix
bodyTransform n = translate n 0 identity

linkTransform :: Link -> Matrix
linkTransform (Link joint body _) = bodyTransform body * jointTransform joint

inverseLinkTransform :: Link -> Matrix
inverseLinkTransform = invert . linkTransform

mapWithMatrix :: (Matrix -> Tree Link -> Link) -> Matrix -> Forest Link -> Forest Link
mapWithMatrix f matrix = map mapNode where
    mapNode (Node link nodes) = Node (f matrix (Node link nodes')) nodes' where
        matrix' = (linkTransform link) * matrix
        nodes' = mapWithMatrix f matrix' nodes
-- }}}

ends :: Matrix -> Tree Link -> [((Double, Double), (Double, Double))]
ends matrix (Node (Link joint body target) nodes) = zip currents $ map (transformPoint $ invert matrix) targets where
    (currents, targets) = unzip $ ends' identity $ Node (Link unrotatedJoint body target) nodes
    unrotatedJoint = case joint of
        (RevoluteJoint min max _) -> RevoluteJoint min max 0
        (PrismaticJoint min max _) -> PrismaticJoint min max 0
ends' matrix (Node link@(Link _ _ target) nodes) = (case target of
    Just target -> (transformPoint matrix' (0, 0), target):pairs
    Nothing -> pairs) where
    pairs = concat $ map (ends' matrix') nodes
    matrix' = (linkTransform link) * matrix

improve :: Forest Link -> Forest Link
improve = mapWithMatrix improveNode identity

improveNode :: Matrix -> Tree Link -> Link
improveNode matrix node@(Node (Link joint body target) nodes) =
    Link (improveJoint (ends matrix node) joint) body target

improveJoint :: [((Double, Double), (Double, Double))] -> Joint -> Joint
improveJoint [] joint = joint
improveJoint ends (RevoluteJoint min max currentAngle) = RevoluteJoint min max $
    maximumBy (compare `on` accuracy) $ min:max:(filter valid $
    map normalizeAngle [theta, theta + pi]) where
    (current, target) = unzip ends
    dot' (x1, y1) = dot (y1, -x1)
    theta = atan $ (sum $ zipWith dot' target current) / (sum $ zipWith dot target current)
    accuracy angle = sum $ zipWith dot target $
        map (transformPoint $ rotate angle identity) current
    valid angle = (angle <= max) && (angle >= min)
improveJoint ends (PrismaticJoint min max _) = PrismaticJoint min max $
    maximumBy (compare `on` accuracy) $ min:max:(filter valid [x]) where
    (current, target) = unzip ends
    difference = sum ((zipWith (-) `on` (map fst)) target current)
    x = difference / (genericLength target)
    accuracy position = 2 * position * difference - (genericLength target) * position * position
    valid position = (position <= max) && (position >= min)

-- Internal functions
normalizeAngle :: Double -> Double
normalizeAngle theta
    | theta < 0 = theta + 2 * pi
    | theta >= 2 * pi = theta - 2 * pi
    | otherwise = theta

-- vim: fdm=marker

