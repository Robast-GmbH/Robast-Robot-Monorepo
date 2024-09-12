import 'package:flutter/material.dart';
import 'package:media_kit/media_kit.dart';
import 'package:media_kit_video/media_kit_video.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class VideoPage extends StatefulWidget {
  const VideoPage({super.key});

  @override
  State<VideoPage> createState() => _VideoPageState();
}

class _VideoPageState extends State<VideoPage> {
  late final player = Player(configuration: const PlayerConfiguration(muted: true));
  late final controller = VideoController(player);

  @override
  void initState() {
    super.initState();
    player.open(Media('asset:///assets/robast_video.mp4'));
  }

  @override
  void dispose() {
    player.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Video',
      child: Center(
          child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 32) + const EdgeInsets.only(bottom: 32, top: 16),
        child: Video(
          controller: controller,
        ),
      )),
    );
  }
}
