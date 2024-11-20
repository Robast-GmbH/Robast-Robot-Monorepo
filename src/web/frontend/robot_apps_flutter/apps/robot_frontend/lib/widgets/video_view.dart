import 'package:flutter/material.dart';
import 'package:media_kit/media_kit.dart';
import 'package:media_kit_video/media_kit_video.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class VideoView extends StatefulWidget {
  const VideoView({required this.path, super.key});
  final String path;
  @override
  State<VideoView> createState() => _VideoViewState();
}

class _VideoViewState extends State<VideoView> {
  late final player = Player(configuration: const PlayerConfiguration(muted: true));
  late final controller = VideoController(player);
  bool videoAvailable = true;
  @override
  void initState() {
    super.initState();
    try {
      player.open(Media('asset:///${widget.path}'));
    } catch (e) {
      videoAvailable = false;
    }
    player.setPlaylistMode(PlaylistMode.loop);
  }

  @override
  void dispose() {
    player.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (videoAvailable) {
      return Video(
        controller: controller,
      );
    } else {
      return const Center(child: Icon(Icons.personal_video, size: 100, color: RobotColors.primaryIcon));
    }
  }
}
