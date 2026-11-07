#!/usr/bin/env python3
"""Backport zenoh #2842 to the exact 1.8.0 router revision used by ROS Lyrical.

The congestion bit indexes pipeline stages, NOT QoS priorities. Pico negotiates
one stage (0), whereas Priority::DEFAULT is Data (5). Refill clears bit 0, so
marking bit 5 on congestion otherwise latches all droppable data off forever.
"""
import argparse
from pathlib import Path

TEST = '''
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn tx_pipeline_congestion_clears_without_qos() -> ZResult<()> {
        let mut config = CONFIG_NOT_STREAMED;
        config.batch.mtu = 256;
        config.batching_enabled = false;
        config.wait_before_drop = Duration::ZERO;
        let priorities = vec![TransportPriorityTx::make(Bits::from(TransportSn::MAX))?];
        let (producer, mut consumer) = TransmissionPipeline::make(config, &priorities);
        let message = NetworkMessage::from(Push {
            wire_expr: "test/no_qos".into(),
            ext_qos: ext::QoSType::new(Priority::Data, CongestionControl::Drop, true),
            ..Push::from(vec![42u8; 64])
        });
        assert!(producer.push_network_message(message.as_ref())?);
        assert!(!producer.push_network_message(message.as_ref())?, "must actually congest");
        let (batch, stage) = timeout(Duration::from_secs(2), consumer.pull()).await?.unwrap();
        consumer.refill(batch, stage);
        assert!(producer.push_network_message(message.as_ref())?,
                "a refilled batch must end congestion without QoS");
        Ok(())
    }

'''


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"Pinned Zenoh source changed; review context: {old[:100]!r}")
    return text.replace(old, new)


def patch(path, regression_only=False):
    text = Path(path).read_text()
    if not regression_only:
        text = replace_once(text,
            'fn set_congested(&self, priority: Priority, status: bool) {\n        let prioflag = 1 << priority as u8;',
            'fn set_congested(&self, idx: usize, status: bool) {\n        let prioflag = 1 << idx as u8;')
        text = replace_once(text,
            'fn is_congested(&self, priority: Priority) -> bool {\n        let prioflag = 1 << priority as u8;',
            'fn is_congested(&self, idx: usize) -> bool {\n        let prioflag = 1 << idx as u8;')
        before, after = text.split('pub(crate) struct TransmissionPipelineConsumer {')
        if before.count('self.status.is_congested(priority)') != 2:
            raise RuntimeError('Unexpected congestion-check call sites')
        before = before.replace('self.status.is_congested(priority)', 'self.status.is_congested(idx)')
        before = replace_once(before, 'self.status.set_congested(priority, true)', 'self.status.set_congested(idx, true)')
        before = replace_once(before, 'self.status.set_congested(priority, false)', 'self.status.set_congested(idx, false)')
        after = replace_once(after, 'self.status.set_congested(priority, false)', 'self.status.set_congested(priority as usize, false)')
        text = before + 'pub(crate) struct TransmissionPipelineConsumer {' + after
    marker = '    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]\n    async fn tx_pipeline_flow()'
    if TEST not in text:
        text = replace_once(text, marker, TEST + marker)
    Path(path).write_text(text)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path)
    parser.add_argument('--regression-only', action='store_true')
    args = parser.parse_args()
    patch(args.path, args.regression_only)
