//! # DMA implimitation for ADC, testing only on esp32c3 only
//!
//! TBD.

use embedded_dma::WriteBuffer;
use fugit::HertzU32;

use super::*;
use crate::{
    clock::Clocks,
    dma::{AdcPeripheral, Channel, ChannelTypes, DmaError, DmaPeripheral, DmaTransfer, RxPrivate},
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
    pub fn dma_read<'t, RXBUF, PIN>(
        &'t mut self,
        _pin: &AdcPin<PIN, ADCI, ()>,
        sample_freq: HertzU32,
        rxbuf: &'t mut RXBUF,
        clocks: &Clocks,
    ) -> Result<AdcDmaTransfer<'t, 'd, ADCI, C, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
        PIN: AdcChannel,
    {
        let (ptr, len) = unsafe { rxbuf.write_buffer() };

        const ADC_LL_CLKM_DIV_NUM_DEFAULT: u8 = 15;
        const ADC_LL_CLKM_DIV_B_DEFAULT: u8 = 1;
        const ADC_LL_CLKM_DIV_A_DEFAULT: u8 = 0;

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
            let atten = self.adc.attenuations[PIN::CHANNEL as usize].unwrap();
            // ADC1 only
            let unit = 0;
            let pattern_val =
                (atten as u32 & 0x3) | ((PIN::CHANNEL as u32 & 0x7) << 2) | ((unit & 0x1) << 5);
            // one pattern only
            let pattern_index = 0;
            let _index = pattern_index / 4;
            let offset = (pattern_index % 4) * 6;
            let mut tab = saradc.sar_patt_tab1().read().bits();
            tab &= !(0xFC0000 >> offset);
            tab |= ((pattern_val & 0x3F) << 18) >> offset;
            saradc
                .sar_patt_tab1()
                .write(|w| unsafe { w.saradc_sar_patt_tab1().bits(tab) });
        }
        let apb_clk_freq = clocks.apb_clock.to_Hz();
        let interval = (apb_clk_freq
            / (ADC_LL_CLKM_DIV_NUM_DEFAULT
                + ADC_LL_CLKM_DIV_A_DEFAULT / ADC_LL_CLKM_DIV_B_DEFAULT
                + 1) as u32
            / 2
            / sample_freq.to_Hz()) as u16;
        // defmt::info!(
        //     "apb_clk_freq: {}, sample_freq: {}, interval: {}, old: {}",
        //     apb_clk_freq,
        //     sample_freq.to_Hz(),
        //     interval,
        //     0b111111111011
        // );
        saradc.ctrl2().modify(|_, w| unsafe {
            // w.saradc_meas_num_limit().set_bit();
            w.saradc_meas_num_limit().clear_bit();
            w.saradc_max_meas_num().bits(10);
            w.saradc_timer_target().bits(interval);
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
        rx.listen_ch_in_done();
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
    pub fn stop(&mut self) -> Result<(), Error> {
        let saradc = &*unsafe { peripherals::APB_SARADC::steal() };
        saradc
            .ctrl2()
            .modify(|_, w| w.saradc_timer_en().clear_bit());
        self.adc_dma.channel.rx.unlisten_ch_in_done();
        self.adc_dma.channel.rx.unlisten_eof();
        Ok(())
    }
}

impl<'t, 'd, ADCI, C: ChannelTypes, DmaMode: Mode> DmaTransfer
    for AdcDmaTransfer<'t, 'd, ADCI, C, DmaMode>
{
    fn wait(mut self) -> Result<(), DmaError> {
        while !self.is_done() {}
        self.stop().map_err(|_| DmaError::Exhausted)?;
        let rx = &self.adc_dma.channel.rx;
        if rx.has_error() || rx.has_dscr_empty_error() || rx.has_eof_error() {
            return Err(DmaError::DescriptorError);
        }
        Ok(())
    }

    fn is_done(&self) -> bool {
        self.adc_dma.channel.rx.is_done()
    }
}
impl<'t, 'd, ADCI, C: ChannelTypes, DmaMode: Mode> Drop
    for AdcDmaTransfer<'t, 'd, ADCI, C, DmaMode>
{
    fn drop(&mut self) {
        self.stop().ok();
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
