//! # DMA implimitation for ADC, testing only on esp32c3 only
//!
//! TBD.

use embedded_dma::WriteBuffer;

use super::*;
use crate::{
    dma::{AdcPeripheral, Channel, ChannelTypes, DmaError, DmaPeripheral, RxPrivate},
    peripherals,
    Mode,
};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    DmaError(DmaError),
    MaxDmaTransferSizeExceeded,
    FifoSizeExeeded,
    Unsupported,
    Unknown,
}

pub struct AdcDma<'d, ADCI, C: ChannelTypes, DmaMode: Mode> {
    adc: ADC<'d, ADCI>,
    channel: Channel<'d, C, DmaMode>,
}

pub trait WithDmaAdc<'d, ADCI, C, DmaMode>
where
    C: ChannelTypes,
    C::P: AdcPeripheral,
    DmaMode: Mode,
{
    fn with_dma(self, channel: Channel<'d, C, DmaMode>) -> AdcDma<'d, ADCI, C, DmaMode>;
}

impl<'d, ADCI, C, DmaMode> WithDmaAdc<'d, ADCI, C, DmaMode> for ADC<'d, ADCI>
where
    C: ChannelTypes,
    C::P: AdcPeripheral,
    DmaMode: Mode,
{
    fn with_dma(self, channel: Channel<'d, C, DmaMode>) -> AdcDma<'d, ADCI, C, DmaMode> {
        AdcDma { adc: self, channel }
    }
}

impl<'d, ADCI, C, DmaMode> AdcDma<'d, ADCI, C, DmaMode>
where
    C: ChannelTypes,
    C::P: AdcPeripheral,
    DmaMode: Mode,
{
    pub fn dma_read<'t, RXBUF>(
        &'t mut self,
        rxbuf: &'t mut RXBUF,
    ) -> Result<AdcDmaTransfer<'t, 'd, ADCI, C, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { rxbuf.write_buffer() };

        const ADC_LL_CLKM_DIV_NUM_DEFAULT: u8 = 15;
        const ADC_LL_CLKM_DIV_B_DEFAULT: u8 = 1;
        const ADC_LL_CLKM_DIV_A_DEFAULT: u8 = 0;

        // let sample_freq_hz = 80_000;
        // let clk_src_freq_hz = 5_000_000;
        // let interval = clk_src_freq_hz
        //     / (ADC_LL_CLKM_DIV_NUM_DEFAULT + ADC_LL_CLKM_DIV_A_DEFAULT /
        // ADC_LL_CLKM_DIV_B_DEFAULT + 1)     / 2
        //     / sample_freq_hz;

        // info!("interval: {}", interval);

        let saradc = &*unsafe { peripherals::APB_SARADC::steal() };
        // stop adc
        saradc
            .ctrl2()
            .modify(|_, w| w.saradc_timer_en().clear_bit());
        saradc.ctrl().modify(|_, w| {
            w.saradc_start_force()
                .clear_bit()
                .saradc_start()
                .clear_bit()
        });
        // setup clocks
        saradc.clkm_conf().modify(|_, w| unsafe {
            w.clkm_div_a()
                .bits(ADC_LL_CLKM_DIV_A_DEFAULT)
                .clkm_div_b()
                .bits(ADC_LL_CLKM_DIV_B_DEFAULT)
                .clkm_div_num()
                .bits(ADC_LL_CLKM_DIV_NUM_DEFAULT)
                .clk_en()
                .set_bit()
        });
        saradc.onetime_sample().modify(|_, w| {
            w.saradc1_onetime_sample().clear_bit();
            w.saradc2_onetime_sample().clear_bit();
            w
        });
        // clear pattern
        saradc
            .ctrl()
            .modify(|_, w| w.saradc_sar_patt_p_clear().set_bit());
        saradc
            .ctrl()
            .modify(|_, w| w.saradc_sar_patt_p_clear().clear_bit());
        let pattern_len = 1;
        saradc
            .ctrl()
            .modify(|_, w| unsafe { w.saradc_sar_patt_len().bits(pattern_len as u8 - 1) });
        {
            // setup patterns
            // typedef struct {
            //     uint8_t atten;      ///< Attenuation of this ADC channel
            //     uint8_t channel;    ///< ADC channel
            //     uint8_t unit;       ///< ADC unit
            //     uint8_t bit_width;  ///< ADC output bit width
            // } adc_digi_pattern_config_t;
            // typedef struct  {
            //     union {
            //         struct {
            //             uint8_t atten:      2;
            //             uint8_t channel:    3;
            //             uint8_t unit:       1;
            //             uint8_t reserved:   2;
            //         };
            //         uint8_t val;
            //     };
            // } __attribute__((packed)) adc_ll_digi_pattern_table_t;
            let pattern = [3u32, 0, 0, 12];
            let pattern_val =
                (pattern[0] & 0x3) | ((pattern[1] & 0x7) << 2) | ((pattern[2] & 0x1) << 5);
            let pattern_index = 0;
            let tab = saradc.sar_patt_tab1().read().bits();
            // defmt::info!("read tab {:x}", tab);
            let _index = pattern_index / 4;
            let offset = (pattern_index % 4) * 6;
            let mut tab = tab;
            tab &= !(0xFC0000 >> offset);
            tab |= ((pattern_val & 0x3F) << 18) >> offset;
            saradc
                .sar_patt_tab1()
                .write(|w| unsafe { w.saradc_sar_patt_tab1().bits(tab) });
            // defmt::info!("set tab to {:x}", tab);
        }
        saradc.ctrl2().modify(|_, w| unsafe {
            // w.saradc_meas_num_limit().set_bit();
            w.saradc_meas_num_limit().clear_bit();
            w.saradc_max_meas_num().bits(10);
            // dump
            w.saradc_timer_target().bits(0b111111111011);
            w
        });
        // reset adc digital controller
        saradc
            .dma_conf()
            .modify(|_, w| w.apb_adc_reset_fsm().set_bit());
        saradc
            .dma_conf()
            .modify(|_, w| w.apb_adc_reset_fsm().clear_bit());
        // set adc eof
        const SOC_ADC_DIGI_DATA_BYTES_PER_CONV: usize = 4;
        saradc.dma_conf().modify(|_, w| unsafe {
            w.apb_adc_eof_num()
                .bits((len / SOC_ADC_DIGI_DATA_BYTES_PER_CONV) as u16)
        });
        let rx = &mut self.channel.rx;
        // start dma
        rx.prepare_transfer_without_start(false, DmaPeripheral::Adc, ptr, len)
            .unwrap();
        rx.listen_eof();
        rx.start_transfer().unwrap();
        // connect DMA and peripheral
        saradc.dma_conf().modify(|_, w| w.apb_adc_trans().set_bit());
        // start ADC
        saradc.ctrl2().modify(|_, w| w.saradc_timer_en().set_bit());
        Ok(AdcDmaTransfer { adc_dma: self })
    }
}

pub struct AdcDmaTransfer<'t, 'd, ADCI, C: ChannelTypes, DmaMode: Mode> {
    adc_dma: &'t mut AdcDma<'d, ADCI, C, DmaMode>,
}

impl<'t, 'd, ADCI, C: ChannelTypes, DmaMode: Mode> AdcDmaTransfer<'t, 'd, ADCI, C, DmaMode> {
    pub fn wait(mut self) -> Result<(), Error> {
        let saradc = &*unsafe { peripherals::APB_SARADC::steal() };
        let rx = &mut self.adc_dma.channel.rx;
        // rx.wait().map_err(|e| Error::DmaError(e))?;
        while !rx.is_done() {}
        // stop adc
        saradc
            .ctrl2()
            .modify(|_, w| w.saradc_timer_en().clear_bit());
        // saradc.ctrl().modify(|_, w| {
        //     w.saradc_start_force()
        //         .clear_bit()
        //         .saradc_start()
        //         .clear_bit()
        // });
        // stop dma
        // rx.stop_transfer().map_err(|e| Error::DmaError(e))?;
        Ok(())
    }
}

// adc_digi_output_data_t
#[derive(Clone, Copy)]
pub struct AdcDigiOutputData(u32);
impl AdcDigiOutputData {
    pub fn data(&self) -> u16 {
        (self.0 & 0xFFF) as u16
    }
    pub fn channel(&self) -> u8 {
        ((self.0 >> 13) & 0x7) as u8
    }
    pub fn unit(&self) -> u8 {
        ((self.0 >> 16) & 0x1) as u8
    }
}
impl From<u32> for AdcDigiOutputData {
    fn from(data: u32) -> Self {
        Self(data)
    }
}
impl From<&[u8]> for AdcDigiOutputData {
    fn from(data: &[u8]) -> Self {
        Self(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }
}
