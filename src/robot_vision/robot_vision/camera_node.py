    def _camera_info_for_frame(
        self,
        frame: np.ndarray,
        stamp,
    ) -> CameraInfo:

        # 원본 템플릿을 수정하지 않고 복사
        info = copy.deepcopy(self._camera_info_template)

        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.height = int(frame.shape[0])
        info.width = int(frame.shape[1])

        # 실제 영상 크기와 보정에 사용한 크기 비교
        same_size = (
            info.width,
            info.height,
        ) == (
            self._image_width,
            self._image_height,
        )

        self._effective_calibrated = (
            self._calibrated
            and same_size
            and self._mode == MODE_REAL
        )

        if not self._effective_calibrated:
            # 해상도가 다르거나 mock 영상이면 보정값 적용 금지
            info.d = [0.0] * 5
            info.k = [0.0] * 9
            info.r = [0.0] * 9
            info.p = [0.0] * 12

        return info