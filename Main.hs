{- inverse_kinematics: Main.hs
 -
 - Copyright (c) 2010 Michael Forney <mforney@berkeley.org>
 -
 - This file is a part of inverse_kinematics.
 -
 - inverse_kinematics is free software; you can redistribute it and/or modify
 - it under the terms of the GNU General Public License version 2, as published
 - by the Free Software Foundation.
 -
 - inverse_kinematics is distributed in the hope that it will be useful, but
 - WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 - or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 - more details.
 -
 - You should have received a copy of the GNU General Public License along with
 - inverse_kinematics.  If not, see <http://www.gnu.org/licenses/>.
 -}

import System.Environment
import System.Directory
import Control.Applicative hiding (many)
import Control.Monad
import Text.Show.Pretty
import Text.Printf
import Text.ParserCombinators.Parsec hiding (label)
import Foreign.Marshal.Error
import Numeric

-- Data Structures
import Data.IORef
import Data.Maybe
import Data.List
import Data.Tree
import Data.Map (Map)
import qualified Data.Map as Map

-- Graphics Imports
import Graphics.UI.Gtk
import Graphics.Rendering.Cairo
import qualified Graphics.Rendering.Cairo.Matrix as M

import InverseKinematics

timelineFrames = 500 :: Int
timelineFrameWidth = 15 :: Int
timelineFrameLabelHeight = 20 :: Int

data UI = UI { uiFocus :: IORef (Zipper Link)
             , uiFocusMatrix :: IORef (Matrix, Matrix)
             , uiDragging :: IORef Bool
             , uiOriginalNodes :: Forest Link

             -- Main Widgets
             , uiWindow :: Window
             , uiDrawingArea :: DrawingArea

             -- Timeline
             , uiAnimation :: IORef (Map Int Int)
             , uiAnimating :: IORef Bool
             , uiKeyFrames :: IORef Int
             , uiTimeline :: DrawingArea
             , uiTimelineCursor :: HScale
             , uiPoses :: ListStore (Int, Forest Link)
             , uiPosesView :: TreeView

             -- Menu Items
             , uiSavePoseItem :: MenuItem
             , uiQuitItem :: MenuItem
             , uiAboutItem :: MenuItem

             -- Buttons
             , uiImproveButton :: Button
             , uiSolveButton :: Button
             , uiResetButton :: Button
             , uiSaveButton :: Button
             , uiUpdateButton :: Button
             , uiRemoveButton :: Button
             , uiAddButton :: Button
             , uiSetButton :: Button
             , uiPlayButton :: Button
             , uiRenderButton :: Button

             -- Properties
             , uiJointProperties :: VBox
             , uiNodeType :: Label
             , uiPosition :: Label
             , uiJointParameter :: Label
             , uiJointMinimum :: Label
             , uiJointMaximum :: Label
             , uiTarget :: Label }

-- Zipper for Data.Tree {{{
data Crumb a = Crumb { crumbValue :: a
                     , crumbLeft :: Forest a
                     , crumbRight :: Forest a } deriving Show
type Zipper a = (Forest a, [Crumb a])

zipper :: Forest Link -> Zipper Link
zipper nodes = (nodes, [])

up :: Zipper Link -> Zipper Link
up (nodes, (Crumb link left right):crumbs) =
    (left ++ [Node link nodes] ++ right, crumbs)

down :: Zipper a -> [Zipper a]
down (nodes, crumbs) = zipWith makeZipper (init $ inits nodes) (init $ tails nodes) where
    makeZipper left ((Node value nodes'):right) = (nodes', (Crumb value left right):crumbs)

isRoot :: Zipper a -> Bool
isRoot = null . snd

root :: Zipper Link -> Forest Link
root = fst . until isRoot up

search :: (Double, Double) -> Forest Link -> Maybe ([Matrix], Zipper Link)
search point = search' [M.identity] point . zipper where
    search' ms@(m:_) point | magnitude (M.transformPoint m point) < 8 = Just . ((,) ms)
    search' ms@(m:_) point = listToMaybe . (mapMaybe search'') . down where
        search'' z@(_, (Crumb link _ _):_) =
            search' ((m * inverseLinkTransform link):ms) point z
    magnitude p = sqrt $ dot p p

value :: Zipper a -> Maybe a
value (_, (Crumb v _ _):_) = Just v
value _ = Nothing

modifyValue :: (a -> a) -> Zipper a -> Zipper a
modifyValue f (nodes, c@(Crumb v _ _):crumbs) = (nodes, c { crumbValue = f v }:crumbs)
modifyValue _ z = z

setValue :: a -> Zipper a -> Zipper a
setValue v (nodes, c:crumbs) = (nodes, c { crumbValue = v }:crumbs)
setValue _ z = z

directions :: Zipper a -> [Int]
directions = map crumbDirection . snd where
    crumbDirection (Crumb _ left _) = length left
-- }}}

fix :: Eq a => (a -> a) -> a -> a
fix f a | f a == a = a
        | otherwise = fix f $ f a

mapPair :: (a -> b) -> (a, a) -> (b, b)
mapPair f (x1, x2) = (f x1, f x2)

-- Rendering Utilities {{{
viewport :: Double -> Double -> Matrix
viewport width height = M.translate (width / 2) (height / 2) $
    M.scale size (-size) M.identity where
    size = (min width height) / 1000

inverseViewport :: Double -> Double -> Matrix
inverseViewport width height = M.invert $ viewport width height

local :: Render a -> Render ()
local = (save >>) . (>> restore)
--- }}}

animationFrame :: UI -> Int -> IO (Maybe (Forest Link))
animationFrame ui n = do
    (before, after) <- (break ((>= n) . fst) . Map.toAscList) <$> readIORef (uiAnimation ui)
    let (frame1, frame2) = mapPair listToMaybe (reverse before, after)
    poses <- Map.fromList <$> listStoreToList (uiPoses ui)
    return $ case (frame1, frame2) of
        (Just (n1, name1), Just (n2, name2)) -> do
            case (Map.lookup name1 poses, Map.lookup name2 poses) of
                (Just pose1, Just pose2) -> Just $ interpolatePose
                    ((fromIntegral $ n - n1) / (fromIntegral $ n2 - n1)) pose1 pose2
                _ -> Nothing
        (Just (_, name), _) -> Map.lookup name poses
        (_, Just (_, name)) -> Map.lookup name poses
        _ -> Nothing

interpolate :: Double -> Double -> Double -> Double
interpolate n x1 x2 = (1 - n) * x1 + n * x2

interpolatePose :: Double -> Forest Link -> Forest Link -> Forest Link
interpolatePose n = zipWith interpolateNode where
    interpolateJoint joint@(RevoluteJoint min max v1) (RevoluteJoint _ _ v2)
        | v2 - v1 < pi = RevoluteJoint min max $ interpolate n v1 v2
        | otherwise = RevoluteJoint min max $ interpolate n v1 (v2 - 2 * pi)
    interpolateJoint joint@(PrismaticJoint min max v1) (PrismaticJoint _ _ v2) =
        PrismaticJoint min max $ interpolate n v1 v2
    interpolateNode :: Tree Link -> Tree Link -> Tree Link
    interpolateNode (Node link@(Link joint1 _ _) nodes1) (Node (Link joint2 _ _) nodes2) =
        Node link' nodes' where
        link' = link { linkJoint = interpolateJoint joint1 joint2 }
        nodes' = zipWith interpolateNode nodes1 nodes2

renderNodes :: Matrix -> Bool -> [Int] -> Forest Link -> Render ()
renderNodes matrix animating directions = (>> renderSocket) . mapM_ renderNode where
    directions' | null directions = []
                | otherwise = tail directions
    renderSocket = local $ do
        case directions of
            [_] -> setSourceRGB 1 0.65 0
            _ -> setSourceRGB 0 0 1
        arc 0 0 8 0 (2 * pi)
        fill
    renderNode (Node (Link joint body target) nodes) = local $ do
        renderJoint joint
        setSourceRGB 1 0 0
        setLineWidth 5
        transform $ jointTransform joint
        moveTo 0 0
        transform $ bodyTransform body
        lineTo 0 0
        stroke
        unless animating $ renderTarget target
        renderNodes matrix animating directions' nodes
    renderTarget (Just (x, y)) = local $ do
        setDash [4, 4] 0
        setSourceRGBA 0 0 0 0.5
        setLineWidth 3
        moveTo 0 0
        setMatrix matrix
        lineTo x y
        stroke
        setSourceRGB 0 1 0
        arc x y 6 0 (2 * pi)
        fill
    renderTarget _ = return ()
    renderJoint (RevoluteJoint min max _) = local $ do
        setSourceRGB 0 0 0
        setLineWidth 2
        arc 0 0 20 min max
        stroke
    renderJoint (PrismaticJoint min max _) = local $ do
        setSourceRGB 0 0 0
        setLineWidth 2
        rectangle min (-5) (max - min) 10
        stroke
        return ()

renderFrame :: Double -> Double -> Bool -> [Int] -> Forest Link -> Render ()
renderFrame width height animating directions nodes = do
    setSourceRGB 1 1 1
    paint
    setMatrix matrix
    renderNodes matrix animating directions nodes
    where matrix = viewport width height

renderTimeline :: (Double, Double) -> Int -> Map Int Int -> Render ()
renderTimeline (width, height) cursor animation = do
    setSourceRGB 0 0 0
    setLineWidth 2
    mapM_ renderFrame [0..timelineFrames]
    moveTo 0 frameY >> lineTo width frameY >> stroke
    where
        frameY = height - fromIntegral timelineFrameLabelHeight
        renderFrame n = local $ do
            translate (fromIntegral $ n * timelineFrameWidth) 0
            when (n == cursor) $ local $
                setSourceRGB 0.5 0.8 1 >>
                rectangle 0 0 (fromIntegral timelineFrameWidth) frameY >>
                fill
            moveTo 0 0
            lineTo 0 frameY
            stroke
            translate 10 height
            rotate (-pi / 2)
            showText $ show n
            moveTo (fromIntegral timelineFrameLabelHeight + 5) 0
            case Map.lookup n animation of
                Just name -> showText $ "Pose " ++ (show name)
                _ -> return ()

updateCanvas :: UI -> EventM EExpose Bool
updateCanvas ui = tryEvent $ liftIO $ do
    focus <- readIORef (uiFocus ui)
    window <- widgetGetDrawWindow (uiDrawingArea ui)
    (width, height) <- mapPair realToFrac <$> drawableGetSize window
    animating <- readIORef (uiAnimating ui)
    renderWithDrawable window $
        renderFrame width height animating (directions focus) (root focus)

updateTimeline :: UI -> EventM EExpose Bool
updateTimeline ui = tryEvent $ liftIO $ do
    window <- widgetGetDrawWindow (uiTimeline ui)
    cursor <- truncate <$> rangeGetValue (uiTimelineCursor ui)
    (width, height) <- mapPair realToFrac <$> drawableGetSize window
    animation <- readIORef (uiAnimation ui)
    renderWithDrawable window $ do
        setSourceRGB 1 1 1
        paint
        renderTimeline (width, height) cursor animation

updatePropertyFields :: UI -> IO ()
updatePropertyFields ui = do
    link <- value <$> readIORef (uiFocus ui)
    widgetSetSensitive (uiJointProperties ui) (isJust link)

updateProperties :: UI -> IO ()
updateProperties ui = do
    maybeLink <- value <$> readIORef (uiFocus ui)
    focusMatrix <- (M.invert . fst) <$> readIORef (uiFocusMatrix ui)
    case maybeLink of
        Just link -> (labelSetText (uiPosition ui) $ showPoint $
            M.transformPoint focusMatrix (0, 0)) >>
            case link of
                Link joint length target -> do
                    widgetSetSensitive (uiTarget ui) (isJust target)
                    case target of
                        Just point -> labelSetText (uiTarget ui) $ showPoint point
                        _ -> return ()
                    case joint of
                        RevoluteJoint min max angle -> do
                            labelSetText (uiNodeType ui) "Revolute Joint"
                            labelSetText (uiJointParameter ui) $ showDouble angle
                            labelSetText (uiJointMinimum ui) $ showDouble min
                            labelSetText (uiJointMaximum ui) $ showDouble max
                        PrismaticJoint min max position -> do
                            labelSetText (uiNodeType ui) "Prismatic Joint"
                            labelSetText (uiJointParameter ui) $ showDouble position
                            labelSetText (uiJointMinimum ui) $ showDouble min
                            labelSetText (uiJointMaximum ui) $ showDouble max
        Nothing -> do
            labelSetText (uiPosition ui) $ showPoint (0, 0)
            labelSetText (uiNodeType ui) "Root Node"
    where
        showDouble x = showFFloat (Just 10) x ""
        showPoint (x, y) = "(" ++ (take 8 $ showDouble x) ++ ", " ++ (take 10 $ showDouble y) ++ ")"

buttonPress :: UI -> EventM EButton Bool
buttonPress ui = tryEvent $ do
    click <- eventClick
    button <- eventButton
    (width, height) <- liftIO $ mapPair realToFrac <$>
        (drawableGetSize =<< widgetGetDrawWindow (uiDrawingArea ui))
    point <- M.transformPoint (inverseViewport width height) <$> eventCoordinates
    case click of
        SingleClick -> case button of
                LeftButton -> liftIO $ do
                    focus <- readIORef $ uiFocus ui
                    print point
                    case search point (root focus) of
                        Just (matrices, newFocus) -> liftIO $ writeIORef (uiFocus ui) newFocus >>
                            writeIORef (uiFocusMatrix ui) (head matrices, head $ tail matrices) >>
                            updatePropertyFields ui >>
                            updateProperties ui >>
                            writeIORef (uiDragging ui) True >>
                            putStrLn "focus changed"
                        _ -> return ()
                    widgetQueueDraw (uiDrawingArea ui)
        ReleaseClick -> liftIO $ writeIORef (uiDragging ui) False >>
            putStrLn "release click"
        DoubleClick -> case button of
                LeftButton -> liftIO $ do
                    focus <- readIORef $ uiFocus ui
                    writeIORef (uiFocus ui) $ modifyValue
                        (\ l -> l { linkTarget = Just point } ) focus
                    widgetQueueDraw (uiDrawingArea ui)
                    updateProperties ui
                RightButton -> liftIO $ do
                    focus <- readIORef $ uiFocus ui
                    writeIORef (uiFocus ui) $ modifyValue
                        (\ l -> l { linkTarget = Nothing } ) focus
                    widgetQueueDraw (uiDrawingArea ui)
                    updateProperties ui

motionNotify :: UI -> EventM EMotion Bool
motionNotify ui = tryEvent $ do
    dragging <- liftIO $ readIORef (uiDragging ui)
    when dragging $ do
        (width, height) <- liftIO $ mapPair realToFrac <$>
            (drawableGetSize =<< widgetGetDrawWindow (uiDrawingArea ui))
        focusMatrix <- liftIO $ snd <$> readIORef (uiFocusMatrix ui)
        point <- M.transformPoint (inverseViewport width height * focusMatrix) <$> eventCoordinates
        liftIO $ do
            focus <- readIORef (uiFocus ui)
            let newFocus = modifyValue (\ l -> l { linkJoint = modifyJoint point (linkBody l) (linkJoint l) } ) focus
            writeIORef (uiFocus ui) newFocus
            writeIORef (uiFocusMatrix ui) (focusMatrix *
                inverseLinkTransform (fromJust $ value newFocus), focusMatrix)
            updateProperties ui
            widgetQueueDraw (uiDrawingArea ui)
    where
        modifyJoint (x, y) _ (RevoluteJoint min max oldAngle) = RevoluteJoint min max angle''
            where
                angle | x /= 0 = atan $ y / x
                      | y >= 0 = pi / 2
                      | otherwise = 3 * pi / 2
                angle' | x >= 0 && y >= 0 = angle
                       | x < 0 = angle + pi
                       | y < 0 = angle + 2 * pi
                angle'' | angle' < min = min
                        | angle' > max = max
                        | otherwise = angle'
        modifyJoint (x, y) length (PrismaticJoint min max _)
            | (x - length) < min = PrismaticJoint min max min
            | (x - length) > max = PrismaticJoint min max max
            | otherwise = PrismaticJoint min max (x - length)

timelineButtonPress :: UI -> EventM EButton Bool
timelineButtonPress ui = tryEvent $ do
    (x, _) <- eventCoordinates
    LeftButton <- eventButton
    click <- eventClick
    let frame = truncate x `div` timelineFrameWidth
    liftIO $ case click of
        SingleClick -> rangeSetValue (uiTimelineCursor ui)
            (fromIntegral frame)
        DoubleClick -> do
            animation <- readIORef (uiAnimation ui)
            writeIORef (uiAnimation ui) (Map.delete frame animation)
            widgetQueueDraw (uiTimeline ui)

-- UI Actions {{{
about :: IO ()
about = do
    dialog <- aboutDialogNew
    set dialog [ aboutDialogName := "Inverse Kinematics"
               , aboutDialogProgramName := "Inverse Kinematics"
               , aboutDialogVersion := "0.0.1"
               , aboutDialogCopyright := "Copyright 2010 Michael Forney"
               , aboutDialogAuthors := ["Michael Forney"] ]
    dialogRun dialog
    widgetDestroy dialog

savePoseToFile :: UI -> IO ()
savePoseToFile ui = do
    dialog <- fileChooserDialogNew (Just "Save Pose") (Just $ uiWindow ui)
        FileChooserActionSave
        [ (stockCancel, ResponseCancel), (stockSave, ResponseOk) ]
    response <- dialogRun dialog
    case response of
        ResponseOk -> do
            maybeFilename <- fileChooserGetFilename dialog
            case maybeFilename of
                (Just filename) -> putStrLn ("Saving pose to: " ++ filename) >>
                    ppShow . root <$> (readIORef $ uiFocus ui) >>= writeFile filename
                _ -> return ()
        _ -> return ()
    widgetDestroy dialog

improveNodes :: UI -> IO ()
improveNodes ui = root <$> readIORef (uiFocus ui) >>=
    writeIORef (uiFocus ui) . zipper . improve >>
    widgetQueueDraw (uiDrawingArea ui)

solveNodes :: UI -> IO ()
solveNodes ui = root <$> readIORef (uiFocus ui) >>=
    writeIORef (uiFocus ui) . zipper . ((!! 1000) . iterate improve) >>
    widgetQueueDraw (uiDrawingArea ui)

resetNodes :: UI -> IO ()
resetNodes ui = writeIORef (uiFocus ui) (zipper (uiOriginalNodes ui)) >>
    widgetQueueDraw (uiDrawingArea ui)

savePose :: UI -> IO ()
savePose ui = do
    nodes <- root <$> readIORef (uiFocus ui)
    number <- readIORef (uiKeyFrames ui)
    writeIORef (uiKeyFrames ui) (number + 1)
    index <- listStoreAppend (uiPoses ui) (number, nodes)
    treeViewSetCursor (uiPosesView ui) [index] Nothing
    when (index == 0) $ widgetSetSensitive (uiUpdateButton ui) True >>
        widgetSetSensitive (uiRemoveButton ui) True

updatePose :: UI -> IO ()
updatePose ui = do
    nodes <- root <$> readIORef (uiFocus ui)
    path <- fst <$> treeViewGetCursor (uiPosesView ui)
    maybeIter <- treeModelGetIter (uiPoses ui) path
    case maybeIter of
        Just iter -> do
            let index = listStoreIterToIndex iter
            number <- fst <$> listStoreGetValue (uiPoses ui) index
            listStoreSetValue (uiPoses ui) index (number, nodes)
        _ -> return ()

removePose :: UI -> IO ()
removePose ui = do
    path <- fst <$> treeViewGetCursor (uiPosesView ui)
    maybeIter <- treeModelGetIter (uiPoses ui) path
    case maybeIter of
        Just iter -> do
            listStoreRemove (uiPoses ui) $ listStoreIterToIndex iter
            empty <- (== 0) <$> listStoreGetSize (uiPoses ui)
            if empty
                then (widgetSetSensitive (uiUpdateButton ui) False >>
                    widgetSetSensitive (uiRemoveButton ui) False)
                else treeViewSetCursor (uiPosesView ui) [0] Nothing
        _ -> return ()

poseChanged :: UI -> IO ()
poseChanged ui = do
    print "pose changed"
    path <- fst <$> treeViewGetCursor (uiPosesView ui)
    maybeIter <- treeModelGetIter (uiPoses ui) path
    case maybeIter of
        Just iter -> do
            (name, nodes) <- listStoreGetValue (uiPoses ui) $ listStoreIterToIndex iter
            writeIORef (uiFocus ui) $ zipper nodes
            widgetQueueDraw (uiDrawingArea ui)

updateJoint :: UI -> IO ()
updateJoint ui = print "updateJoint"

updateTarget :: UI -> IO ()
updateTarget ui = print "updateTarget"

timelineCursorChanged :: UI -> IO ()
timelineCursorChanged ui = do
    cursor <- truncate <$> rangeGetValue (uiTimelineCursor ui)
    maybePose <- animationFrame ui cursor
    case maybePose of
        Just pose -> writeIORef (uiFocus ui) (zipper pose) >>
            widgetQueueDraw (uiDrawingArea ui)
        Nothing -> return ()
    widgetQueueDraw (uiTimeline ui)

animationAddPose :: UI -> IO ()
animationAddPose ui = do
    path <- fst <$> treeViewGetCursor (uiPosesView ui)
    maybeIter <- treeModelGetIter (uiPoses ui) path
    case maybeIter of
        Just iter -> do
            name <- fst <$> (listStoreGetValue (uiPoses ui) $ listStoreIterToIndex iter)
            cursor <- truncate <$> rangeGetValue (uiTimelineCursor ui)
            animation <- readIORef (uiAnimation ui)
            writeIORef (uiAnimation ui) $ Map.insert cursor name animation
    widgetQueueDraw (uiTimeline ui)

animationSetPose :: UI -> IO ()
animationSetPose ui = savePose ui >> animationAddPose ui

animationPlay :: UI -> IO ()
animationPlay ui = writeIORef (uiAnimating ui) True >>
    timeoutAdd (nextFrame ui) 40 >> return ()

animationRender :: UI -> IO ()
animationRender ui = do
    dialog <- dialogNew

    nameBox <- hBoxNew False 5
    nameLabel <- labelNew $ Just "<b>Animation Name</b>: "
    nameEntry <- entryNew
    boxPackStart nameBox nameLabel PackGrow 0
    boxPackStart nameBox nameEntry PackNatural 0

    widthBox <- hBoxNew False 5
    widthLabel <- labelNew $ Just "<b>Width</b>: "
    widthEntry <- spinButtonNewWithRange 100 10000 100
    spinButtonSetValue widthEntry 500
    boxPackStart widthBox widthLabel PackGrow 0
    boxPackStart widthBox widthEntry PackNatural 0

    heightBox <- hBoxNew False 5
    heightLabel <- labelNew $ Just "<b>Height</b>: "
    heightEntry <- spinButtonNewWithRange 100 10000 100
    spinButtonSetValue heightEntry 500
    boxPackStart heightBox heightLabel PackGrow 0
    boxPackStart heightBox heightEntry PackNatural 0

    destinationBox <- hBoxNew False 5
    destinationLabel <- labelNew $ Just "<b>Destination</b>: "
    destinationEntry <- entryNew
    browseButton <- buttonNewWithMnemonic "_Browse..."
    getCurrentDirectory >>= entrySetText destinationEntry
    boxPackStart destinationBox destinationLabel PackNatural 0
    boxPackStart destinationBox destinationEntry PackGrow 0
    boxPackStart destinationBox browseButton PackNatural 0

    box <- dialogGetUpper dialog
    boxPackStart box nameBox PackNatural 0
    boxPackStart box widthBox PackNatural 0
    boxPackStart box heightBox PackNatural 0
    boxPackStart box destinationBox PackNatural 0

    on browseButton buttonActivated $ do
        destinationDialog <- fileChooserDialogNew (Just "Choose Folder") (Just $ uiWindow ui)
            FileChooserActionSelectFolder
            [ (stockCancel, ResponseCancel), (stockSave, ResponseOk) ]
        response <- dialogRun destinationDialog
        case response of
            ResponseOk -> do
                maybeFolder <- fileChooserGetFilename destinationDialog
                case maybeFolder of
                    (Just folder) -> entrySetText destinationEntry folder
                    _ -> return ()
            _ -> return ()
        widgetDestroy destinationDialog

    dialogAddButton dialog stockOk ResponseOk
    dialogAddButton dialog stockCancel ResponseCancel

    mapM_ (flip labelSetUseMarkup True)
        [nameLabel, widthLabel, heightLabel, destinationLabel]
    mapM_ (flip (flip miscSetAlignment 0) 0.5)
        [nameLabel, widthLabel, heightLabel, destinationLabel]

    widgetShowAll dialog

    response <- dialogRun dialog
    case response of
        ResponseOk -> do
            width <- truncate <$> spinButtonGetValue widthEntry
            height <- truncate <$> spinButtonGetValue heightEntry
            name <- entryGetText nameEntry
            destination <- entryGetText destinationEntry

            animation <- readIORef (uiAnimation ui)
            if Map.null animation
                then return ()
                else do
                    surface <- createImageSurface FormatRGB24 width height
                    let { frames = fst $ Map.findMax animation ;
                        frame n = do
                            pose <- animationFrame ui n
                            renderWith surface $ renderFrame (fromIntegral width) (fromIntegral height)
                                True [] $ fromJust pose
                            surfaceWriteToPNG surface $ destination ++ "/" ++ name ++ "_" ++
                                (printf "%03d" n) ++ ".png"
                        }
                    mapM_ frame [0..frames]
        _ -> return ()

    widgetDestroy dialog

nextFrame :: UI -> IO Bool
nextFrame ui = do
    animation <- readIORef (uiAnimation ui)
    let { end = writeIORef (uiAnimating ui) False >> widgetQueueDraw (uiDrawingArea ui) >>
        return False }
    if Map.null animation
        then end
        else do
            let n = fst $ Map.findMax animation
            cursor <- truncate <$> rangeGetValue (uiTimelineCursor ui)
            if (cursor < n)
                then rangeSetValue (uiTimelineCursor ui) (fromIntegral $ cursor + 1) >>
                    return True
                else end
-- }}}

buildGUI :: IO UI
buildGUI = do
    nodes <- read <$> (readFile =<< head <$> getArgs)

    -- Simple
    {-
    let { nodes =
        [ Node (Link (RevoluteJoint 0 (2 * pi) (pi / 3)) 150 Nothing)
            [ Node (Link (RevoluteJoint 0 (2 * pi) (11 * pi / 6)) 100 Nothing)
                [ Node (Link (RevoluteJoint 0 (2 * pi) (pi / 2)) 100 Nothing) [] ] ] ]
        }
    -}

    -- Combination
    {-
    let { nodes =
        [ Node (Link (RevoluteJoint 0 (2 * pi / 3) (pi / 6)) 150 Nothing)
            [ Node (Link (RevoluteJoint 0 (2 * pi) 4.5) 100 Nothing)
                [ Node (Link (PrismaticJoint 0 100 25) 100 Nothing)
                    [ Node (Link (RevoluteJoint 0 (2 * pi) (5 * pi / 3)) 100 Nothing)
                        [ Node (Link (RevoluteJoint 0 (2 * pi) (pi / 3)) 75 (Just (200, 300)))
                            [] ]
                    , Node (Link (RevoluteJoint 0 (2 * pi) (pi / 2)) 100 Nothing)
                        [ Node (Link (RevoluteJoint 0 (2 * pi) (3 * pi / 2)) 150 Nothing)
                            [ Node (Link (RevoluteJoint 0 (2 * pi) 2.4) 75 (Just (50, 200))) [] ]
                        , Node (Link (RevoluteJoint 0 (2 * pi) (2 * pi / 3)) 100 Nothing)
                            [ Node (Link (RevoluteJoint 0 (2 * pi) 4.4) 100 (Just (-50, 350))) [] ] ] ] ] ] ]
        }
    -}

    -- Basic 1
    {-
    let nodes = [ Node (Link (RevoluteJoint 0 (2 * pi) (-0.5)) 200 Nothing)
                    [ Node (Link (RevoluteJoint 0 (2 * pi) (0.25)) 200 (Just (50, 350)))
                        [ Node (Link (RevoluteJoint 0 (2 * pi) (2)) 200 Nothing)
                            [ Node (Link (RevoluteJoint 0 (2 * pi) (1)) 100 (Just (300, 300)))
                                [] ]
                        , Node (Link (RevoluteJoint 0 (2 * pi) (4)) 200 Nothing)
                            [ Node (Link (RevoluteJoint 0 (2 * pi) (1.5)) 100 (Just (-150, 450)))
                                [] ] ] ]
                , Node (Link (PrismaticJoint 0 100 25) 100 Nothing)
                    [ Node (Link (RevoluteJoint 0 (2 * pi) 1) 200 (Just (150, -150))) [] ] ]
    -}

    -- Humanoid
    {-
    let { nodes =
        [ Node (Link (RevoluteJoint (pi / 6) (5 * pi / 6) (pi / 2)) 100 Nothing)
            [ Node (Link (RevoluteJoint 0 (2 * pi) (pi / 2)) 75 Nothing)
                [ Node (Link (RevoluteJoint 0 (2 * pi) (pi / 3)) 100 Nothing) [] ]
            , Node (Link (RevoluteJoint 0 (2 * pi) (3 * pi / 2)) 75 Nothing)
                [ Node (Link (RevoluteJoint 0 (2 * pi) (5 * pi / 3)) 100 Nothing) [] ]
            , Node (Link (RevoluteJoint 0 (2 * pi) 0) 75 Nothing) [] ]
        , Node (Link (RevoluteJoint 0 (2 * pi) (5 * pi / 3)) 100 Nothing)
            [ Node (Link (RevoluteJoint 0 (2 * pi) (11 * pi / 6)) 100 Nothing) [] ]
        , Node (Link (RevoluteJoint 0 (2 * pi) (4 * pi / 3)) 100 Nothing)
            [ Node (Link (RevoluteJoint 0 (2 * pi) (pi / 6)) 100 Nothing) [] ] ]
        }
    -}

    putStrLn $ drawForest $ map (show <$>) nodes

    focus <- newIORef $ zipper nodes
    focusMatrix <- newIORef (M.identity, M.identity)
    dragging <- newIORef False
    window <- windowNew
    drawingArea <- drawingAreaNew

    savePoseItem <- menuItemNewWithMnemonic "_Save Pose"
    quitItem <- menuItemNewWithMnemonic "_Quit"
    aboutItem <- menuItemNewWithMnemonic "_About"

    improveButton <- buttonNewWithMnemonic "_Improve"
    solveButton <- buttonNewWithMnemonic "_Solve"
    resetButton <- buttonNewWithMnemonic "_Reset"

    jointProperties <- vBoxNew False 5
    nodeType <- labelNew Nothing
    position <- labelNew Nothing
    jointParameter <- labelNew Nothing
    jointMinimum <- labelNew Nothing
    jointMaximum <- labelNew Nothing
    target <- labelNew Nothing

    saveButton <- buttonNewWithMnemonic "_Save"
    updateButton <- buttonNewWithMnemonic "_Update"
    removeButton <- buttonNewWithMnemonic "_Remove"

    addButton <- buttonNewWithLabel "<-"
    setButton <- buttonNewWithLabel "->"
    playButton <- buttonNewWithMnemonic "_Play"
    renderButton <- buttonNewWithMnemonic "_Render"

    animation <- newIORef Map.empty
    animating <- newIORef False
    keyFrames <- newIORef 1
    timeline <- drawingAreaNew
    timelineCursor <- hScaleNewWithRange 0 (fromIntegral timelineFrames) 1

    poses <- listStoreNew [(0, nodes)]
    posesView <- treeViewNewWithModel poses

    let ui = UI
            focus focusMatrix dragging nodes
            -- Main Widgets
            window drawingArea
            -- Timeline
            animation animating keyFrames timeline timelineCursor poses posesView
            -- Menu Items
            savePoseItem quitItem aboutItem
            -- Buttons
            improveButton solveButton resetButton
            saveButton updateButton removeButton
            addButton setButton playButton renderButton
            -- Properties
            jointProperties
            nodeType position jointParameter jointMinimum jointMaximum target

    updateProperties ui
    updatePropertyFields ui

    menuBar <- do
        bar <- menuBarNew
        fileMenu <- menuNew
        helpMenu <- menuNew

        mapM_ (menuShellAppend fileMenu) [savePoseItem, quitItem]
        mapM_ (menuShellAppend helpMenu) [aboutItem]

        fileItem <- menuItemNewWithMnemonic "_File"
        menuItemSetSubmenu fileItem fileMenu

        helpItem <- menuItemNewWithMnemonic "_Help"
        menuItemSetSubmenu helpItem helpMenu

        menuShellAppend bar fileItem
        menuShellAppend bar helpItem

        return bar

    canvas <- do
        frame <- frameNew
        widgetAddEvents drawingArea [Button1MotionMask]
        containerAdd frame drawingArea
        return frame

    properties <- do
        box <- hBoxNew False 5

        -- Buttons
        buttonBox <- vButtonBoxNew
        buttonBoxSetLayout buttonBox ButtonboxStart

        boxPackStart buttonBox improveButton PackNatural 0
        boxPackStart buttonBox solveButton PackNatural 0
        boxPackStart buttonBox resetButton PackNatural 0

        -- Tab Bar
        notebook <- notebookNew

        -- Properties
        propertiesBox <- hBoxNew False 5
        column1SpacerBox <- vBoxNew False 5
        column1Table <- tableNew 2 2 False
        separator <- vSeparatorNew
        column2Table <- tableNew 2 4 False

        -- Column 1
        nodeTypeFrame <- frameNew
        nodeTypeLabel <- labelNew $ Just "<b>Node Type</b>: "
        labelSetUseMarkup nodeTypeLabel True
        miscSetAlignment nodeTypeLabel 0 0.5
        set nodeTypeFrame [ containerChild := nodeType ]

        positionFrame <- frameNew
        positionLabel <- labelNew $ Just "<b>Position</b>: "
        labelSetUseMarkup positionLabel True
        miscSetAlignment positionLabel 0 0.5
        set positionFrame [ containerChild := position ]

        tableAttachDefaults column1Table nodeTypeLabel 0 1 0 1
        tableAttachDefaults column1Table nodeTypeFrame 1 2 0 1
        tableAttachDefaults column1Table positionLabel 0 1 1 2
        tableAttachDefaults column1Table positionFrame 1 2 1 2

        boxPackStart column1SpacerBox column1Table PackNatural 0

        -- Column 2
        jointParameterFrame <- frameNew
        jointParameterLabel <- labelNew $ Just "<b>Joint Parameter</b>: "
        labelSetUseMarkup jointParameterLabel True
        miscSetAlignment jointParameterLabel 0 0.5
        set jointParameterFrame [ containerChild := jointParameter ]

        jointMinimumFrame <- frameNew
        jointMinimumLabel <- labelNew $ Just "<b>Joint Minimum</b>: "
        labelSetUseMarkup jointMinimumLabel True
        miscSetAlignment jointMinimumLabel 0 0.5
        set jointMinimumFrame [ containerChild := jointMinimum ]

        jointMaximumFrame <- frameNew
        jointMaximumLabel <- labelNew $ Just "<b>Joint Maximum</b>: "
        labelSetUseMarkup jointMaximumLabel True
        miscSetAlignment jointMaximumLabel 0 0.5
        set jointMaximumFrame [ containerChild := jointMaximum ]

        targetFrame <- frameNew
        targetLabel <- labelNew $ Just "<b>Target</b>: "
        labelSetUseMarkup targetLabel True
        miscSetAlignment targetLabel 0 0.5
        set targetFrame [ containerChild := target ]

        tableAttachDefaults column2Table jointParameterLabel 0 1 0 1
        tableAttachDefaults column2Table jointParameterFrame 1 2 0 1
        tableAttachDefaults column2Table jointMinimumLabel 0 1 1 2
        tableAttachDefaults column2Table jointMinimumFrame 1 2 1 2
        tableAttachDefaults column2Table jointMaximumLabel 0 1 2 3
        tableAttachDefaults column2Table jointMaximumFrame 1 2 2 3
        tableAttachDefaults column2Table targetLabel 0 1 3 4
        tableAttachDefaults column2Table targetFrame 1 2 3 4

        boxPackStart jointProperties column2Table PackNatural 0

        boxPackStart propertiesBox column1SpacerBox PackGrow 0
        boxPackStart propertiesBox separator PackNatural 0
        boxPackStart propertiesBox jointProperties PackGrow 0

        -- Timeline
        animationButtonBox <- vButtonBoxNew
        buttonBoxSetLayout animationButtonBox ButtonboxStart

        boxPackStart animationButtonBox addButton PackNatural 0
        boxPackStart animationButtonBox setButton PackNatural 0
        boxPackStart animationButtonBox playButton PackNatural 0
        boxPackStart animationButtonBox renderButton PackNatural 0

        posesWindow <- scrolledWindowNew Nothing Nothing
        scrolledWindowSetPolicy posesWindow PolicyNever PolicyAutomatic
        containerAdd posesWindow posesView

        poseColumn <- treeViewColumnNew
        renderer <- cellRendererTextNew
        treeViewColumnPackStart poseColumn renderer True
        cellLayoutSetAttributes poseColumn renderer poses $ \row ->
            [cellText := "Pose " ++ (show $ fst row)]

        treeViewSetHeadersVisible posesView False
        treeViewAppendColumn posesView poseColumn
        treeViewSetCursor (uiPosesView ui) [0] Nothing

        poseButtonBox <- vButtonBoxNew
        buttonBoxSetLayout poseButtonBox ButtonboxStart

        boxPackStart poseButtonBox saveButton PackNatural 0
        boxPackStart poseButtonBox updateButton PackNatural 0
        boxPackStart poseButtonBox removeButton PackNatural 0

        timelineBox <- hBoxNew False 5

        timelineWindowBox <- vBoxNew False 5

        timelineWindow <- scrolledWindowNew Nothing Nothing
        widgetSetSizeRequest timeline (timelineFrameWidth * timelineFrames)
            timelineFrameLabelHeight
        scrolledWindowAddWithViewport timelineWindow timeline
        scrolledWindowSetPolicy timelineWindow PolicyAutomatic PolicyNever
        scaleSetValuePos timelineCursor PosRight

        boxPackStart timelineWindowBox timelineWindow PackGrow 0
        boxPackStart timelineWindowBox timelineCursor PackNatural 0

        boxPackStart timelineBox timelineWindowBox PackGrow 0
        boxPackStart timelineBox animationButtonBox PackNatural 0
        boxPackStart timelineBox posesWindow PackNatural 0
        boxPackStart timelineBox poseButtonBox PackNatural 0

        notebookAppendPage notebook propertiesBox "Node Properties"
        notebookAppendPage notebook timelineBox "Timeline"

        boxPackStart box buttonBox PackNatural 0
        boxPackStart box notebook PackGrow 0

        return box

    layout <- do
        box <- vBoxNew False 0
        paned <- vPanedNew
        panedPack1 paned canvas True False
        panedPack2 paned properties False True
        boxPackStart box menuBar PackNatural 0
        boxPackStart box paned PackGrow 0
        return box

    containerAdd window layout
    widgetShowAll window

    return ui

connectGUI :: UI -> IO ()
connectGUI ui = do
    on (uiWindow ui) objectDestroy mainQuit
    on (uiDrawingArea ui) exposeEvent $ updateCanvas ui
    on (uiDrawingArea ui) buttonPressEvent $ buttonPress ui
    on (uiDrawingArea ui) buttonReleaseEvent $ buttonPress ui
    on (uiDrawingArea ui) motionNotifyEvent $ motionNotify ui

    on (uiSavePoseItem ui) menuItemActivate $ savePoseToFile ui
    on (uiQuitItem ui) menuItemActivate mainQuit
    on (uiAboutItem ui) menuItemActivate about

    on (uiImproveButton ui) buttonActivated $ improveNodes ui
    on (uiSolveButton ui) buttonActivated $ solveNodes ui
    on (uiResetButton ui) buttonActivated $ resetNodes ui

    on (uiSaveButton ui) buttonActivated $ savePose ui
    on (uiUpdateButton ui) buttonActivated $ updatePose ui
    on (uiRemoveButton ui) buttonActivated $ removePose ui

    on (uiAddButton ui) buttonActivated $ animationAddPose ui
    on (uiSetButton ui) buttonActivated $ animationSetPose ui
    on (uiPlayButton ui) buttonActivated $ animationPlay ui
    on (uiRenderButton ui) buttonActivated $ animationRender ui

    on (uiPosesView ui) cursorChanged $ poseChanged ui

    on (uiTimeline ui) exposeEvent $ updateTimeline ui
    on (uiTimeline ui) buttonPressEvent $ timelineButtonPress ui
    on (uiTimelineCursor ui) valueChanged $ timelineCursorChanged ui

    return ()


main :: IO ()
main = initGUI >> buildGUI >>= connectGUI >> mainGUI

-- vim: fdm=marker

