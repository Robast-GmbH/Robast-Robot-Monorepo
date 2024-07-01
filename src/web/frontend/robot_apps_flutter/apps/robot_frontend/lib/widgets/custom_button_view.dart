import 'package:flutter/material.dart';

class CustomButtonView extends StatelessWidget {
  const CustomButtonView({
    required this.text,
    required this.onPressed,
    super.key,
  });
  final String text;
  final VoidCallback onPressed;
  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onPressed,
      child: Padding(
        padding: const EdgeInsets.all(4),
        child: Container(
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.bottomCenter,
              end: Alignment.topCenter,
              colors: [
                Colors.white.withOpacity(0.5),
                Colors.white.withOpacity(0.3),
              ],
            ),
            borderRadius: BorderRadius.circular(12),
          ),
          child: SizedBox.expand(
            child: Align(
              child: Text(
                text,
                textAlign: TextAlign.center,
                style: const TextStyle(
                  height: 0,
                  color: Colors.white,
                  fontSize: 40,
                  fontWeight: FontWeight.w400,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
